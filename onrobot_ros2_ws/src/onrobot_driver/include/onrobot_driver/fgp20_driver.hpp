#ifndef ONROBOT_DRIVER__FGP20_DRIVER_HPP_
#define ONROBOT_DRIVER__FGP20_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/fg_status.hpp"
#include "onrobot_msgs/msg/fg_command.hpp"

namespace onrobot_driver
{

class FGP20Driver : public OnRobotBase
{
public:
  FGP20Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address);
  ~FGP20Driver() = default;

  bool initialize() override;
  bool update() override;
  
  // FG gripper commands
  bool grip(float width_mm, float force_n, float speed_percent = 100.0f);
  bool stop();
  
  // VG vacuum commands
  bool setVacuum(uint8_t vacuum_percent, bool ignore = false);
  bool vacuumOn(uint8_t vacuum_percent = 80);
  bool vacuumOff();
  
  // Combined grip with vacuum
  bool gripWithVacuum(float width_mm, float force_n, uint8_t vacuum_percent);
  
  // Status reading
  bool readStatus(onrobot_msgs::msg::FGStatus& status);

private:
  // 2FGP20 register addresses
  static constexpr uint16_t REG_VG_COMMAND = 0x0001;      // Vacuum command
  static constexpr uint16_t REG_FG_WIDTH = 0x0002;        // Target width (1/10 mm)
  static constexpr uint16_t REG_FG_FORCE = 0x0003;        // Target force (1/10 N)
  static constexpr uint16_t REG_FG_SPEED = 0x0004;        // Speed (1/10 %)
  static constexpr uint16_t REG_FG_COMMAND = 0x0005;      // Gripper command
  
  static constexpr uint16_t REG_STATUS = 0x0100;          // Status
  static constexpr uint16_t REG_MAX_FORCE = 0x0102;       // Max force
  static constexpr uint16_t REG_EXTERNAL_WIDTH = 0x0103;  // Current width
  static constexpr uint16_t REG_MIN_WIDTH = 0x0105;       // Min width
  static constexpr uint16_t REG_MAX_WIDTH = 0x0106;       // Max width
  static constexpr uint16_t REG_CURRENT_FORCE = 0x0109;   // Current force
  static constexpr uint16_t REG_VG_VACUUM = 0x010A;       // Vacuum level
  
  static constexpr uint16_t REG_MOVING_FINGER_LENGTH = 0x0400;
  static constexpr uint16_t REG_MOVING_FINGER_HEIGHT = 0x0401;
  static constexpr uint16_t REG_MOVING_FINGERTIP_OFFSET = 0x0402;
  static constexpr uint16_t REG_FIXED_FINGER_LENGTH = 0x0403;
  static constexpr uint16_t REG_VACUUM_CUPS_OFFSET = 0x0407;
  
  // Commands
  static constexpr uint16_t CMD_NONE = 0x0000;
  static constexpr uint16_t CMD_GRIP_EXTERNAL = 0x0001;
  static constexpr uint16_t CMD_STOP = 0x0003;
  
  // Status bits
  static constexpr uint16_t STATUS_BUSY = 0x0001;
  static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
  static constexpr uint16_t STATUS_FORCE_GRIP = 0x0008;
  
  float max_width_;
  float max_force_;
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__FGP20_DRIVER_HPP_
