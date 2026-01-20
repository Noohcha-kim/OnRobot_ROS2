#ifndef ONROBOT_DRIVER__FG3_DRIVER_HPP_
#define ONROBOT_DRIVER__FG3_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/fg3_status.hpp"
#include "onrobot_msgs/msg/fg3_command.hpp"

namespace onrobot_driver
{

class FG3Driver : public OnRobotBase
{
public:
  FG3Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address);
  ~FG3Driver() = default;

  bool initialize() override;
  bool update() override;
  
  // 3FG commands
  bool grip(float diameter_mm, float force_percent, uint8_t grip_type = 0);
  bool move(float diameter_mm, uint8_t grip_type = 0);
  bool stop();
  bool flexibleGrip(float diameter_mm, float force_percent, uint8_t grip_type = 0);
  
  // Status reading
  bool readStatus(onrobot_msgs::msg::FG3Status& status);

private:
  // 3FG15/3FG25 register addresses (from Connectivity Guide v1.22)
  static constexpr uint16_t REG_TARGET_FORCE = 0x0000;    // Force 10*% (Write) - 0-1000
  static constexpr uint16_t REG_TARGET_DIAMETER = 0x0001; // Diameter 0.1mm (Write)
  static constexpr uint16_t REG_GRIP_TYPE = 0x0002;       // 0=external, 1=internal (Write)
  static constexpr uint16_t REG_COMMAND = 0x0003;         // Command (Write)
  
  static constexpr uint16_t REG_STATUS = 0x0100;                    // Status bits (Read)
  static constexpr uint16_t REG_RAW_DIAMETER = 0x0101;              // Raw diameter 0.1mm (Read)
  static constexpr uint16_t REG_DIAMETER_WITH_OFFSET = 0x0102;      // Diameter with offset 0.1mm (Read)
  static constexpr uint16_t REG_FORCE_APPLIED = 0x0103;             // Force applied N (Read)
  static constexpr uint16_t REG_MINIMUM_DIAMETER = 0x0201;          // Min diameter 0.1mm (Read)
  static constexpr uint16_t REG_MAXIMUM_DIAMETER = 0x0202;          // Max diameter 0.1mm (Read)
  
  // Commands
  static constexpr uint16_t CMD_GRIP = 0x0001;
  static constexpr uint16_t CMD_MOVE = 0x0002;
  static constexpr uint16_t CMD_STOP = 0x0004;
  static constexpr uint16_t CMD_FLEXIBLE_GRIP = 0x0005;
  
  // Grip types
  static constexpr uint16_t GRIP_TYPE_EXTERNAL = 0x0000;
  static constexpr uint16_t GRIP_TYPE_INTERNAL = 0x0001;
  
  // Status bits
  static constexpr uint16_t STATUS_BUSY = 0x0001;
  static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
  static constexpr uint16_t STATUS_FORCE_GRIP_DETECTED = 0x0004;
  
  // Model-specific parameters (set during initialization)
  float max_diameter_;
  float max_force_;
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__FG3_DRIVER_HPP_
