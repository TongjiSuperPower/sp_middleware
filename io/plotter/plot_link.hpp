#ifndef SP__PLOT_LINK_HPP
#define SP__PLOT_LINK_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

// Wire format for sp::SuperPlotter: named topics, subscription only. Not NetworkTables.
//
// Board -> host  AA BC | id u8 | name, NUL          once per topic, when first flushed
// Host  -> board AA BD | op u8 | name, NUL          1 sub, 2 unsub, 3 all, 4 clear
// Board -> host  AA BB | count u8 | (id u8, f32le)* | xor u8
//                xor covers every preceding byte. A frame that fails it is dropped.
//
// Registered topics and transmitted values are separate limits. A topic costs
// RAM only until the host subscribes to it; the frame is built from the
// subscribed subset and split when it does not fit in one frame, so the number
// of registered topics is bounded by RAM and the u8 id space, not by a frame.
//
// Paths are limited to 15 characters (PLOT_MAX_NAME = 16 including the NUL).
// SuperPlotter::record() and sp::record() only accept literals and reject an
// over-long one at compile time; nothing is ever truncated at runtime.

#if defined(__GNUC__)
#define SP_PLOT_BARRIER() __asm volatile("" ::: "memory")
#else
#define SP_PLOT_BARRIER() ((void)0)
#endif

// How many topics can be registered at once. Bounded by the u8 id space (255)
// and by RAM: each slot is 24 bytes, so 128 slots cost 3 KB.
// Override with -DSP_PLOT_MAX_TOPICS=<n> if the board needs another number.
#ifndef SP_PLOT_MAX_TOPICS
#define SP_PLOT_MAX_TOPICS 128
#endif

namespace sp
{
constexpr uint8_t PLOT_OP_SUB = 1;
constexpr uint8_t PLOT_OP_UNSUB = 2;
constexpr uint8_t PLOT_OP_ALL = 3;
constexpr uint8_t PLOT_OP_CLEAR = 4;

constexpr int PLOT_MAX_TOPICS = SP_PLOT_MAX_TOPICS;
constexpr int PLOT_MAX_NAME = 16;  // 15 characters plus the NUL
constexpr int PLOT_MAX_ID = 255;

// True when a literal path fits in PlotSlot::name (N includes the NUL).
// Paths shorter than 15 characters bind to the const char (&)[N] overloads of
// SuperPlotter::record() / publish(), which static_assert this. Runtime names
// are only used by internal tables that already checked their literals, and
// insert() rejects an over-long one instead of truncating it.
template <size_t N>
constexpr bool plot_name_fits(const char (&)[N])
{
  return N <= PLOT_MAX_NAME;
}

struct PlotSlot
{
  char name[PLOT_MAX_NAME];
  float value;
  uint16_t hash;
  uint8_t subscribed;
  uint8_t announced;
};

class PlotPublisher
{
public:
  void set(float value) const
  {
    if (slot_ != nullptr) slot_->value = value;
  }

  explicit operator bool() const { return slot_ != nullptr; }

private:
  friend class TopicTable;
  PlotSlot * slot_ = nullptr;
};

// One row of a table polled on a fixed cadence: a path, the value it should
// read, and the publisher handle that was registered for it. Kept here rather
// than in super_plotter.hpp so the host-side bridge can share the table type
// without pulling in the HAL.
struct PolledRow
{
  const char * name;
  float (*read)();
  PlotPublisher pub;
};

class TopicTable
{
public:
  static constexpr int kMaxTopics = PLOT_MAX_TOPICS;

  // Called only while a topic is being registered, so record() on an already
  // known topic stays lock-free. Leave unset on a single threaded build.
  // The lock returns an opaque state that the unlock restores.
  using LockFn = uint32_t (*)();
  using UnlockFn = void (*)(uint32_t);

  void set_lock(LockFn lock, UnlockFn unlock)
  {
    lock_ = lock;
    unlock_ = unlock;
  }

  PlotPublisher publish(const char * name)
  {
    PlotPublisher out;
    if (name == nullptr || name[0] == '\0') return out;
    const uint16_t hash = name_hash(name);
    PlotSlot * slot = lookup(name, hash);
    if (slot == nullptr) {
      const uint32_t state = lock();
      slot = lookup(name, hash);
      if (slot == nullptr) slot = insert(name, hash);
      unlock(state);
    }
    out.slot_ = slot;
    return out;
  }

  // Writes one value, registering the topic the first time the name is seen.
  // Safe to call from any task: the cold path is serialized, the hot path is
  // a hash compare plus a single float store.
  void record(const char * name, float value)
  {
    if (name == nullptr || name[0] == '\0') return;
    const uint16_t hash = name_hash(name);
    PlotSlot * slot = lookup(name, hash);
    if (slot != nullptr) {
      slot->value = value;
      return;
    }
    const uint32_t state = lock();
    slot = lookup(name, hash);
    if (slot == nullptr) slot = insert(name, hash);
    if (slot != nullptr) slot->value = value;
    unlock(state);
  }

  void ingest(const uint8_t * data, size_t n)
  {
    for (size_t i = 0; i < n; ++i) {
      if (acc_len_ < static_cast<int>(sizeof acc_)) acc_[acc_len_++] = data[i];
      pump();
    }
  }

  // Packs as many pending announcements as fit in dst. Does not mark them
  // sent: commit_announce() does that after the bytes have actually gone out,
  // so a transmit that is cut short is retried instead of forgotten.
  size_t pop_announce(uint8_t * dst, size_t cap)
  {
    if (dst == nullptr) return 0;
    size_t total = 0;
    const int n = count_;
    for (int i = 0; i < n; ++i) {
      if (topics_[i].name[0] == '\0' || topics_[i].announced) continue;
      const size_t name_n = std::strlen(topics_[i].name);
      const size_t need = 3 + name_n + 1;
      if (total + need > cap || i > PLOT_MAX_ID) break;
      dst[total + 0] = 0xAA;
      dst[total + 1] = 0xBC;
      dst[total + 2] = static_cast<uint8_t>(i);
      std::memcpy(dst + total + 3, topics_[i].name, name_n + 1);
      total += need;
    }
    return total;
  }

  // Marks the announcements packed by the last pop_announce() of `n` bytes.
  // `n` must be a prefix of what pop_announce() just wrote.
  void commit_announce(size_t n)
  {
    size_t total = 0;
    const int count = count_;
    for (int i = 0; i < count && total < n; ++i) {
      if (topics_[i].name[0] == '\0' || topics_[i].announced) continue;
      const size_t name_n = std::strlen(topics_[i].name);
      const size_t need = 3 + name_n + 1;
      if (total + need > n || i > PLOT_MAX_ID) break;
      topics_[i].announced = 1;
      total += need;
    }
  }

  // Packs subscribed topics into one frame, resuming at the cursor so a set
  // larger than one frame is sent over consecutive calls instead of dropped.
  size_t pop_sample(uint8_t * dst, size_t cap)
  {
    if (dst == nullptr || cap < 9) return 0;
    const int n = count_;
    if (n == 0) return 0;

    size_t total = 3;
    int packed = 0;
    int i = sample_cursor_;
    if (i >= n) i = 0;
    for (int scanned = 0; scanned < n; ++scanned) {
      if (topics_[i].subscribed) {
        if (total + 5 + 1 > cap) break;
        dst[total++] = static_cast<uint8_t>(i);
        uint32_t bits = 0;
        std::memcpy(&bits, &topics_[i].value, sizeof bits);
        dst[total++] = static_cast<uint8_t>(bits);
        dst[total++] = static_cast<uint8_t>(bits >> 8);
        dst[total++] = static_cast<uint8_t>(bits >> 16);
        dst[total++] = static_cast<uint8_t>(bits >> 24);
        ++packed;
      }
      if (++i >= n) i = 0;
    }
    sample_cursor_ = i;
    if (packed == 0) return 0;
    dst[0] = 0xAA;
    dst[1] = 0xBB;
    dst[2] = static_cast<uint8_t>(packed);
    uint8_t sum = 0;
    for (size_t k = 0; k < total; ++k) sum ^= dst[k];
    dst[total++] = sum;
    return total;
  }

  int size() const { return count_; }
  int subscribed_count() const
  {
    int c = 0;
    const int n = count_;
    for (int i = 0; i < n; ++i) {
      if (topics_[i].subscribed) ++c;
    }
    return c;
  }

private:
  PlotSlot topics_[kMaxTopics]{};
  volatile int count_ = 0;
  int sample_cursor_ = 0;
  uint8_t acc_[64]{};
  int acc_len_ = 0;
  LockFn lock_ = nullptr;
  UnlockFn unlock_ = nullptr;

  static uint16_t name_hash(const char * s)
  {
    uint32_t h = 2166136261u;
    while (*s != '\0') {
      h ^= static_cast<uint8_t>(*s++);
      h *= 16777619u;
    }
    return static_cast<uint16_t>(h ^ (h >> 16));
  }

  uint32_t lock() const { return lock_ == nullptr ? 0 : lock_(); }
  void unlock(uint32_t state) const
  {
    if (unlock_ != nullptr) unlock_(state);
  }

  PlotSlot * lookup(const char * name, uint16_t hash)
  {
    if (name == nullptr) return nullptr;
    const int n = count_;
    for (int i = 0; i < n; ++i) {
      if (topics_[i].hash == hash && std::strcmp(topics_[i].name, name) == 0) return &topics_[i];
    }
    return nullptr;
  }

  // Appends a slot. The name is fully written before the slot becomes visible
  // through count_, so a reader either misses the topic or sees it complete.
  // A name that does not fit is rejected rather than truncated: truncation
  // could merge two distinct paths into one field.
  PlotSlot * insert(const char * name, uint16_t hash)
  {
    if (name == nullptr || name[0] == '\0' || count_ >= kMaxTopics) return nullptr;
    const size_t len = std::strlen(name);
    if (len >= static_cast<size_t>(PLOT_MAX_NAME)) return nullptr;
    PlotSlot & slot = topics_[count_];
    std::memset(&slot, 0, sizeof slot);
    std::memcpy(slot.name, name, len + 1);
    slot.hash = hash;
    SP_PLOT_BARRIER();
    count_ = count_ + 1;
    return &slot;
  }

  int find(const char * name) const
  {
    const uint16_t hash = name_hash(name);
    for (int i = 0; i < count_; ++i) {
      if (topics_[i].hash == hash && std::strcmp(topics_[i].name, name) == 0) return i;
    }
    return -1;
  }

  void apply(uint8_t op, const char * name)
  {
    if (op == PLOT_OP_ALL || op == PLOT_OP_CLEAR) {
      const uint8_t on = op == PLOT_OP_ALL ? 1 : 0;
      for (int i = 0; i < count_; ++i) {
        topics_[i].subscribed = on;
        topics_[i].announced = 0;
      }
      sample_cursor_ = 0;
      return;
    }
    const int id = find(name);
    if (id < 0) return;
    if (op == PLOT_OP_SUB) topics_[id].subscribed = 1;
    else if (op == PLOT_OP_UNSUB) topics_[id].subscribed = 0;
  }

  void pump()
  {
    while (acc_len_ >= 3) {
      int start = -1;
      for (int i = 0; i + 1 < acc_len_; ++i) {
        if (acc_[i] == 0xAA && acc_[i + 1] == 0xBD) {
          start = i;
          break;
        }
      }
      if (start < 0) {
        acc_len_ = acc_[acc_len_ - 1] == 0xAA ? 1 : 0;
        if (acc_len_ == 1) acc_[0] = 0xAA;
        return;
      }
      if (start > 0) {
        std::memmove(acc_, acc_ + start, static_cast<size_t>(acc_len_ - start));
        acc_len_ -= start;
      }
      const uint8_t op = acc_[2];
      int end = 3;
      if (op == PLOT_OP_SUB || op == PLOT_OP_UNSUB) {
        while (end < acc_len_ && acc_[end] != 0) ++end;
        if (end >= acc_len_) return;
      }
      char name[PLOT_MAX_NAME];
      name[0] = '\0';
      if (op == PLOT_OP_SUB || op == PLOT_OP_UNSUB) {
        const int c = end - 3;
        // A name that does not fit cannot match any registered topic (they are
        // all shorter), so drop it instead of truncating it into a mismatch.
        if (c <= 0 || c >= PLOT_MAX_NAME) {
          ++end;
          std::memmove(acc_, acc_ + end, static_cast<size_t>(acc_len_ - end));
          acc_len_ -= end;
          continue;
        }
        std::memcpy(name, acc_ + 3, static_cast<size_t>(c));
        name[c] = '\0';
        ++end;
      }
      apply(op, name);
      std::memmove(acc_, acc_ + end, static_cast<size_t>(acc_len_ - end));
      acc_len_ -= end;
    }
  }
};

}  // namespace sp

#endif  // SP__PLOT_LINK_HPP
