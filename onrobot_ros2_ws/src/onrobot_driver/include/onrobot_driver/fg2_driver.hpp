#ifndef ONROBOT_DRIVER__FG2_DRIVER_HPP_
#define ONROBOT_DRIVER__FG2_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/fg_status.hpp"
#include "onrobot_msgs/msg/fg_command.hpp"

namespace onrobot_driver
{

class FG2Driver : public OnRobotBase
{
public:
  FG2Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address);
  ~FG2Driver() = default;

  bool initialize() override;
  bool update() override;
  
  // 2FG commands
  bool grip(float width_mm, float force_n);
  bool stop();
  
  // Status reading
  bool readStatus(onrobot_msgs::msg::FGStatus& status);

private:
  // 2FG7/2FG14 register addresses (from Connectivity Guide v1.22)
  static constexpr uint16_t REG_TARGET_WIDTH = 0x0000;   // Width 0.1mm (Write)
  static constexpr uint16_t REG_TARGET_FORCE = 0x0001;   // Force N (Write)
  static constexpr uint16_t REG_TARGET_SPEED = 0x0002;   // Speed % (Write)
  static constexpr uint16_t REG_COMMAND = 0x0003;        // Command (Write)
  
  static constexpr uint16_t REG_STATUS = 0x0100;              // Status bits (Read)
  static constexpr uint16_t REG_EXTERNAL_WIDTH = 0x0101;      // External width 0.1mm (Read)
  static constexpr uint16_t REG_INTERNAL_WIDTH = 0x0102;      // Internal width 0.1mm (Read)
  static constexpr uint16_t REG_MIN_EXTERNAL_WIDTH = 0x0103;  // Min external 0.1mm (Read)
  static constexpr uint16_t REG_MAX_EXTERNAL_WIDTH = 0x0104;  // Max external 0.1mm (Read)
  static constexpr uint16_t REG_MIN_INTERNAL_WIDTH = 0x0105;  // Min internal 0.1mm (Read)
  static constexpr uint16_t REG_MAX_INTERNAL_WIDTH = 0x0106;  // Max internal 0.1mm (Read)
  static constexpr uint16_t REG_FORCE = 0x0107;               // Current force N (Read)
  
  // Commands
  static constexpr uint16_t CMD_GRIP_EXTERNAL = 0x0001;
  static constexpr uint16_t CMD_GRIP_INTERNAL = 0x0002;
  static constexpr uint16_t CMD_STOP = 0x0003;
  
  // Status bits
  static constexpr uint16_t STATUS_BUSY = 0x0001;
  static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
  
  // Model-specific parameters (set during initialization)
  float max_width_;
  float max_force_;
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__FG2_DRIVER_HPP_
