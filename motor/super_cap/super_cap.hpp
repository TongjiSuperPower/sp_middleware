#ifndef SP__SUPERCAP_HPP
#define SP__SUPERCAP_HPP

#include <cstdint>

namespace sp
{
enum class SuperCapMode
{
  AUTOMODE = 0x00,     // �Զ�ģʽ��Ĭ�ϣ�
  DISOUTPUT,           // ֻ�䲻��
  DISCHARGE,           // ֻ�Ų���
  DISCHARGE_DISOUTPUT  // ���䲻��
};

constexpr float CAPACITANCE = 4.5454f;  // ��������

class SuperCap
{
public:
  const uint16_t rx_id = 0x301;  // ���ݷ���֡ID
  const uint16_t tx_id = 0x300;  // ���ݿ���֡ID

  float power_in;   // �������빦��
  float power_out;  // ���ݳ�ŵ繦��
  float voltage;    // ���ݵ�ѹ���ŵ�3V��Ȼ��3v->6V����ٴηŵ磩

  float cap_energy;

  SuperCap(SuperCapMode mode_ = SuperCapMode::AUTOMODE);

  bool is_alive(uint32_t now_ms) const;

  void read(uint8_t * data, uint32_t stamp_ms);
  void write(uint8_t * data, uint16_t power_limit, uint16_t buffer_energy, uint8_t chassis_output);

private:
  SuperCapMode mode_;

  uint8_t temputer_;  // �¶�(�ֽ׶κ㶨Ϊ25)
  uint8_t status_;    // ״̬��־λ���ֽ׶α���Ϊ0x10��

  uint32_t last_read_ms_;  // ��һ�ζ�ȡ���ݵ�ʱ���
};

}  // namespace sp

#endif  // IO_SUPERCAP_HPP