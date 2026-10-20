#ifndef SP__PLOT_LINK_HPP
#define SP__PLOT_LINK_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

// Wire format for sp::SuperPlotter: named topics, subscription only. Not NetworkTables.
//
// Board -> host  AA BC | id u8 | name, NUL | unit u8   once per topic, when first flushed
// Host  -> board AA BD | op u8 | name, NUL          1 sub, 2 unsub, 3 all, 4 clear
// Board -> host  AA BB | count u8 | (id u8, f32le)* | xor u8
//                xor covers every preceding byte. A frame that fails it is dropped.
//
// Matrices are a second frame family, because a 24-byte scalar slot cannot hold
// a variable number of cells:
//
// Board -> host  AA BE | id u8 | rows u8 | cols u8 | name, NUL | unit u8
// Board -> host  AA BF | count u8 | (id u8, rows u8, cols u8, f32le[rows*cols])* | xor u8
//
// Cells are row-major. rows and cols are repeated in every sample frame so a
// host that missed an announcement can still parse the payload. Subscriptions
// are the same AA BD names as scalars; a name belongs to one of the two tables.
//
// The unit is one byte from PlotUnit, sent right after the name: a label for
// the host's axis, not a conversion. The value is always transmitted in the
// unit it was recorded in. Adding it costs one byte per topic, once.
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

// Matrices are stored separately from scalar topics: a matrix slot holds the
// whole cell block, so a bigger PLOT_MATRIX_CELLS multiplies RAM. 8 slots of
// 32 cells cost about 1.2 KB. One frame carries at most 32 floats, which is
// 135 bytes and fits in PLOT_TX_SIZE. Override with -DSP_PLOT_MAX_MATRICES=<n>
// and -DSP_PLOT_MATRIX_CELLS=<n> if the board can afford more.
#ifndef SP_PLOT_MAX_MATRICES
#define SP_PLOT_MAX_MATRICES 8
#endif
#ifndef SP_PLOT_MATRIX_CELLS
#define SP_PLOT_MATRIX_CELLS 32
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

// Unit label carried by an announcement. One byte on the wire and one byte in
// the slot, so a topic still costs 24 bytes. The ids are part of the protocol:
// the host keeps the same table, so never renumber an existing entry -- add new
// ones at the end.
//
// Only units the dashboard can actually convert are listed. A quantity with no
// group there (torque in N*m, mass in kg, power in W ...) should be recorded
// with PlotUnit::None: a label the host cannot resolve is worse than no label,
// because it looks like the axis was handled.
//
// None means "unknown or dimensionless"; the host shows the field without a
// unit instead of guessing one from the path.
enum class PlotUnit : uint8_t
{
  None = 0,

  // length
  Meter = 1,
  Millimeter = 2,
  Centimeter = 3,
  Inch = 4,
  Foot = 5,

  // angle
  Radian = 6,
  Degree = 7,
  Revolution = 8,

  // time
  Second = 9,
  Millisecond = 10,
  Microsecond = 11,
  Minute = 12,

  // linear
  MeterPerSecond = 13,
  MeterPerSecondSquared = 14,

  // angular
  RadianPerSecond = 15,
  DegreePerSecond = 16,
  RadianPerSecondSquared = 17,

  // electrical
  Ampere = 18,
  Volt = 19,
  Joule = 20,

  // thermal
  Celsius = 21,
  Kelvin = 22,

  // misc
  Hertz = 23,
  RevolutionPerMinute = 24,
};

constexpr int PLOT_UNIT_COUNT = 25;

constexpr int PLOT_MAX_MATRICES = SP_PLOT_MAX_MATRICES;
constexpr int PLOT_MATRIX_MAX_CELLS = SP_PLOT_MATRIX_CELLS;

// A matrix slot owns its cells: publishing copies them, so a caller may pass a
// stack buffer that goes away. Rows and cols are fixed by the first
// registration, like the unit.
struct PlotMatrixSlot
{
  char name[PLOT_MAX_NAME];
  uint16_t hash;
  uint8_t rows;
  uint8_t cols;
  PlotUnit unit;
  uint8_t flags;
  float cells[PLOT_MATRIX_MAX_CELLS];
};

class MatrixPublisher
{
public:
  // `values` must hold rows * cols floats in row-major order.
  void set(const float * values) const
  {
    if (slot_ == nullptr || values == nullptr) return;
    const size_t count = static_cast<size_t>(slot_->rows) * slot_->cols;
    std::memcpy(slot_->cells, values, count * sizeof(float));
  }

  uint8_t rows() const { return slot_ == nullptr ? 0 : slot_->rows; }
  uint8_t cols() const { return slot_ == nullptr ? 0 : slot_->cols; }

  explicit operator bool() const { return slot_ != nullptr; }

private:
  friend class TopicTable;
  PlotMatrixSlot * slot_ = nullptr;
};

// One row of a matrix table polled on a fixed cadence. `fill` writes
// rows * cols floats in row-major order into the provided buffer.
struct PolledMatrix
{
  const char * name;
  uint8_t rows;
  uint8_t cols;
  PlotUnit unit;
  void (*fill)(float * out);
  MatrixPublisher pub;
};

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
  PlotUnit unit;
  // Bit 0 subscribed, bit 1 announced. Packed so the whole slot stays 24 bytes:
  // 16 + 4 + 2 + 1 + 1.
  uint8_t flags;
};

// The unit byte must not cost RAM: keep it in the same 24-byte slot as before.
static_assert(sizeof(PlotSlot) == 24, "PlotSlot grew; check the field packing");

constexpr uint8_t PLOT_SLOT_SUBSCRIBED = 1u << 0;
constexpr uint8_t PLOT_SLOT_ANNOUNCED = 1u << 1;

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
  PlotUnit unit;
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

  PlotPublisher publish(const char * name, PlotUnit unit = PlotUnit::None)
  {
    PlotPublisher out;
    if (name == nullptr || name[0] == '\0') return out;
    const uint16_t hash = name_hash(name);
    PlotSlot * slot = lookup(name, hash);
    if (slot == nullptr) {
      const uint32_t state = lock();
      slot = lookup(name, hash);
      if (slot == nullptr) slot = insert(name, hash, unit);
      unlock(state);
    }
    out.slot_ = slot;
    return out;
  }

  // Writes one value, registering the topic the first time the name is seen.
  // Safe to call from any task: the cold path is serialized, the hot path is
  // a hash compare plus a single float store.
  //
  // The unit only matters on the registering call; a later call with a
  // different unit is ignored, so one topic never changes its label mid-run.
  void record(const char * name, float value, PlotUnit unit = PlotUnit::None)
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
    if (slot == nullptr) slot = insert(name, hash, unit);
    if (slot != nullptr) slot->value = value;
    unlock(state);
  }

  // Registers a matrix topic and returns a handle. Publishing copies the cells,
  // so the caller can reuse its buffer. Both dimensions must be non-zero and
  // rows * cols must fit in PLOT_MATRIX_MAX_CELLS.
  MatrixPublisher publish_matrix(
    const char * name, uint8_t rows, uint8_t cols, PlotUnit unit = PlotUnit::None)
  {
    MatrixPublisher out;
    if (name == nullptr || name[0] == '\0' || !matrix_dims_ok(rows, cols)) return out;
    const uint16_t hash = name_hash(name);
    PlotMatrixSlot * slot = lookup_matrix(name, hash);
    if (slot == nullptr) {
      const uint32_t state = lock();
      slot = lookup_matrix(name, hash);
      if (slot == nullptr) slot = insert_matrix(name, hash, rows, cols, unit);
      unlock(state);
    }
    out.slot_ = slot;
    return out;
  }

  // Registers on the first call, then copies the cells. An existing topic keeps
  // its shape: a call with different dimensions is ignored rather than writing
  // past the slot or silently reinterpreting the data.
  void record_matrix(
    const char * name, uint8_t rows, uint8_t cols, const float * values,
    PlotUnit unit = PlotUnit::None)
  {
    MatrixPublisher pub = publish_matrix(name, rows, cols, unit);
    if (pub.slot_ != nullptr && pub.rows() == rows && pub.cols() == cols) pub.set(values);
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
      if (topics_[i].name[0] == '\0' || (topics_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t name_n = std::strlen(topics_[i].name);
      const size_t need = announce_size(topics_[i]);
      if (total + need > cap || i > PLOT_MAX_ID) break;
      dst[total + 0] = 0xAA;
      dst[total + 1] = 0xBC;
      dst[total + 2] = static_cast<uint8_t>(i);
      std::memcpy(dst + total + 3, topics_[i].name, name_n + 1);
      dst[total + 3 + name_n + 1] = static_cast<uint8_t>(topics_[i].unit);
      total += need;
    }

    // Matrices follow the scalars. The id spaces are separate on the wire, so
    // the slot index is used as-is.
    const int matrix_n = matrix_count_;
    for (int i = 0; i < matrix_n; ++i) {
      if (matrices_[i].name[0] == '\0' || (matrices_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t need = matrix_announce_size(matrices_[i]);
      if (total + need > cap) break;
      dst[total + 0] = 0xAA;
      dst[total + 1] = 0xBE;
      dst[total + 2] = static_cast<uint8_t>(i);
      dst[total + 3] = matrices_[i].rows;
      dst[total + 4] = matrices_[i].cols;
      const size_t name_n = std::strlen(matrices_[i].name);
      std::memcpy(dst + total + 5, matrices_[i].name, name_n + 1);
      dst[total + 5 + name_n + 1] = static_cast<uint8_t>(matrices_[i].unit);
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
      if (topics_[i].name[0] == '\0' || (topics_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t need = announce_size(topics_[i]);
      if (total + need > n || i > PLOT_MAX_ID) break;
      topics_[i].flags |= PLOT_SLOT_ANNOUNCED;
      total += need;
    }

    const int matrix_n = matrix_count_;
    for (int i = 0; i < matrix_n && total < n; ++i) {
      if (matrices_[i].name[0] == '\0' || (matrices_[i].flags & PLOT_SLOT_ANNOUNCED)) continue;
      const size_t need = matrix_announce_size(matrices_[i]);
      if (total + need > n) break;
      matrices_[i].flags |= PLOT_SLOT_ANNOUNCED;
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
      if (topics_[i].flags & PLOT_SLOT_SUBSCRIBED) {
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

  // Packs subscribed matrices into one AA BF frame. Each entry repeats rows and
  // cols: a host that missed the announcement can still parse the payload, and
  // a re-registered topic with new dimensions cannot desynchronise the parser.
  // The cursor makes a set that does not fit span several frames.
  size_t pop_matrix_sample(uint8_t * dst, size_t cap)
  {
    if (dst == nullptr || cap < 11) return 0;
    const int n = matrix_count_;
    if (n == 0) return 0;

    size_t total = 3;
    int packed = 0;
    int i = matrix_cursor_;
    if (i >= n) i = 0;
    for (int scanned = 0; scanned < n; ++scanned) {
      if (matrices_[i].flags & PLOT_SLOT_SUBSCRIBED) {
        const size_t cells = static_cast<size_t>(matrices_[i].rows) * matrices_[i].cols;
        if (total + 3 + cells * 4 + 1 > cap) break;
        dst[total++] = static_cast<uint8_t>(i);
        dst[total++] = matrices_[i].rows;
        dst[total++] = matrices_[i].cols;
        for (size_t c = 0; c < cells; ++c) {
          uint32_t bits = 0;
          std::memcpy(&bits, &matrices_[i].cells[c], sizeof bits);
          dst[total++] = static_cast<uint8_t>(bits);
          dst[total++] = static_cast<uint8_t>(bits >> 8);
          dst[total++] = static_cast<uint8_t>(bits >> 16);
          dst[total++] = static_cast<uint8_t>(bits >> 24);
        }
        ++packed;
      }
      if (++i >= n) i = 0;
    }
    matrix_cursor_ = i;
    if (packed == 0) return 0;
    dst[0] = 0xAA;
    dst[1] = 0xBF;
    dst[2] = static_cast<uint8_t>(packed);
    uint8_t sum = 0;
    for (size_t k = 0; k < total; ++k) sum ^= dst[k];
    dst[total++] = sum;
    return total;
  }

  int size() const { return count_; }
  int matrix_size() const { return matrix_count_; }
  int subscribed_count() const
  {
    int c = 0;
    const int n = count_;
    for (int i = 0; i < n; ++i) {
      if (topics_[i].flags & PLOT_SLOT_SUBSCRIBED) ++c;
    }
    return c;
  }

private:
  PlotSlot topics_[kMaxTopics]{};
  volatile int count_ = 0;
  int sample_cursor_ = 0;
  PlotMatrixSlot matrices_[PLOT_MAX_MATRICES]{};
  volatile int matrix_count_ = 0;
  int matrix_cursor_ = 0;
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
  PlotSlot * insert(const char * name, uint16_t hash, PlotUnit unit)
  {
    if (name == nullptr || name[0] == '\0' || count_ >= kMaxTopics) return nullptr;
    const size_t len = std::strlen(name);
    if (len >= static_cast<size_t>(PLOT_MAX_NAME)) return nullptr;
    PlotSlot & slot = topics_[count_];
    std::memset(&slot, 0, sizeof slot);
    std::memcpy(slot.name, name, len + 1);
    slot.hash = hash;
    slot.unit = unit;
    SP_PLOT_BARRIER();
    count_ = count_ + 1;
    return &slot;
  }

  // 3 header bytes, the name with its NUL, and the unit byte.
  static size_t announce_size(const PlotSlot & slot)
  {
    return 3 + std::strlen(slot.name) + 1 + 1;
  }

  static bool matrix_dims_ok(uint8_t rows, uint8_t cols)
  {
    return rows > 0 && cols > 0 &&
           static_cast<int>(rows) * static_cast<int>(cols) <= PLOT_MATRIX_MAX_CELLS;
  }

  PlotMatrixSlot * lookup_matrix(const char * name, uint16_t hash)
  {
    if (name == nullptr) return nullptr;
    const int n = matrix_count_;
    for (int i = 0; i < n; ++i) {
      if (matrices_[i].hash == hash && std::strcmp(matrices_[i].name, name) == 0) return &matrices_[i];
    }
    return nullptr;
  }

  PlotMatrixSlot * insert_matrix(
    const char * name, uint16_t hash, uint8_t rows, uint8_t cols, PlotUnit unit)
  {
    if (name == nullptr || name[0] == '\0' || matrix_count_ >= PLOT_MAX_MATRICES) return nullptr;
    if (!matrix_dims_ok(rows, cols)) return nullptr;
    const size_t len = std::strlen(name);
    if (len >= static_cast<size_t>(PLOT_MAX_NAME)) return nullptr;
    PlotMatrixSlot & slot = matrices_[matrix_count_];
    std::memset(&slot, 0, sizeof slot);
    std::memcpy(slot.name, name, len + 1);
    slot.hash = hash;
    slot.rows = rows;
    slot.cols = cols;
    slot.unit = unit;
    SP_PLOT_BARRIER();
    matrix_count_ = matrix_count_ + 1;
    return &slot;
  }

  // 3 header bytes + rows + cols + the name with its NUL + the unit byte.
  static size_t matrix_announce_size(const PlotMatrixSlot & slot)
  {
    return 7 + std::strlen(slot.name);
  }

  int find_matrix(const char * name) const
  {
    const uint16_t hash = name_hash(name);
    for (int i = 0; i < matrix_count_; ++i) {
      if (matrices_[i].hash == hash && std::strcmp(matrices_[i].name, name) == 0) return i;
    }
    return -1;
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
      const uint8_t on = op == PLOT_OP_ALL ? PLOT_SLOT_SUBSCRIBED : 0;
      for (int i = 0; i < count_; ++i) {
        // Force a re-announce so a host that just connected learns the units too.
        topics_[i].flags = on;
      }
      for (int i = 0; i < matrix_count_; ++i) {
        matrices_[i].flags = on;
      }
      sample_cursor_ = 0;
      matrix_cursor_ = 0;
      return;
    }
    const int id = find(name);
    if (id >= 0) {
      if (op == PLOT_OP_SUB) topics_[id].flags |= PLOT_SLOT_SUBSCRIBED;
      else if (op == PLOT_OP_UNSUB) topics_[id].flags &= static_cast<uint8_t>(~PLOT_SLOT_SUBSCRIBED);
      return;
    }
    // Scalar and matrix names share the subscribe namespace but not the table.
    const int matrix_id = find_matrix(name);
    if (matrix_id < 0) return;
    if (op == PLOT_OP_SUB) matrices_[matrix_id].flags |= PLOT_SLOT_SUBSCRIBED;
    else if (op == PLOT_OP_UNSUB) matrices_[matrix_id].flags &= static_cast<uint8_t>(~PLOT_SLOT_SUBSCRIBED);
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
