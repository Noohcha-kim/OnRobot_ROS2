#ifndef ONROBOT_DRIVER__HEX_DRIVER_HPP_
#define ONROBOT_DRIVER__HEX_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/hex_wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace onrobot_driver
{

class HEXDriver : public OnRobotBase
{
public:
  HEXDriver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address);
  ~HEXDriver() = default;

  bool initialize() override;
  bool update() override;
  
  // HEX commands
  bool zero();  // Zero/bias the sensor
  
  // Status reading
  bool readWrench(onrobot_msgs::msg::HEXWrench& wrench);

private:
  // HEX-E/H register addresses (from Connectivity Guide v1.22)
  static constexpr uint16_t REG_ZERO = 0x0000;       // Zero/Bias command (R/W)
  static constexpr uint16_t REG_STATUS = 0x0101;     // Status (Read)
  
  static constexpr uint16_t REG_FX = 0x0103;  // Force X (int16, 0.1 N)
  static constexpr uint16_t REG_FY = 0x0104;  // Force Y (int16, 0.1 N)
  static constexpr uint16_t REG_FZ = 0x0105;  // Force Z (int16, 0.1 N)
  static constexpr uint16_t REG_TX = 0x0106;  // Torque X (int16, 0.001 Nm)
  static constexpr uint16_t REG_TY = 0x0107;  // Torque Y (int16, 0.001 Nm)
  static constexpr uint16_t REG_TZ = 0x0108;  // Torque Z (int16, 0.001 Nm)
  
  // Commands
  static constexpr uint16_t CMD_ZERO = 0x0001;
  static constexpr uint16_t CMD_UN_ZERO = 0x0000;
  
  // Convert signed 16-bit register to int16_t
  int16_t toInt16(uint16_t value);
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__HEX_DRIVER_HPP_
