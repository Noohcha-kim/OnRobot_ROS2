#ifndef ONROBOT_DRIVER__MG10_DRIVER_HPP_
#define ONROBOT_DRIVER__MG10_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/mg10_status.hpp"
#include "onrobot_msgs/msg/mg10_command.hpp"

namespace onrobot_driver
{

class MG10Driver : public OnRobotBase
{
public:
  MG10Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address);
  ~MG10Driver() = default;

  bool initialize() override;
  bool update() override;
  
  // MG10 commands
  bool disengage();
  bool engage(uint8_t strength = 100);
  bool engageSmartGrip(uint8_t strength = 100);
  bool autoCalibrate();
  
  // Status reading
  bool readStatus(onrobot_msgs::msg::MG10Status& status);

private:
  // MG10 register addresses (from Connectivity Guide v1.22)
  static constexpr uint16_t REG_COMMAND = 0x0000;           // Control (Write)
  static constexpr uint16_t REG_TARGET_STRENGTH = 0x0001;   // Strength % (Write)
  
  static constexpr uint16_t REG_STATUS = 0x0100;            // Status bits (Read)
  static constexpr uint16_t REG_ERROR_CODE = 0x0101;        // Error code bits (Read)
  static constexpr uint16_t REG_MAGNET_STRENGTH = 0x0102;   // Current strength % (Read)
  
  static constexpr uint16_t REG_FINGER_HEIGHT = 0x0401;     // Finger height 0.1mm (R/W)
  static constexpr uint16_t REG_FINGER_TYPE = 0x0402;       // Finger type (R/W)
  
  // Status bits (REG_STATUS 0x0100)
  static constexpr uint16_t STATUS_PART_GRIPPED = 0x0001;
  static constexpr uint16_t STATUS_NEAR_PART = 0x0002;
  static constexpr uint16_t STATUS_BUSY = 0x0004;
  static constexpr uint16_t STATUS_STRENGTH_NOT_REACHED = 0x0008;
  static constexpr uint16_t STATUS_SMART_GRIP_AVAILABLE = 0x0010;
  static constexpr uint16_t STATUS_SMART_GRIP_FAILED = 0x0020;
  static constexpr uint16_t STATUS_PART_DROPPED = 0x0040;
  static constexpr uint16_t STATUS_TEMP_WARNING = 0x0080;
  
  // Error code bits (REG_ERROR_CODE 0x0101)
  static constexpr uint16_t ERROR_OVER_HEATING = 0x0001;
  static constexpr uint16_t ERROR_SENSOR_TARGET_MISMATCH = 0x0002;
  static constexpr uint16_t ERROR_NO_MOTOR_CALIBRATION = 0x0004;
  static constexpr uint16_t ERROR_NO_MAGNET_CALIBRATION = 0x0008;
  static constexpr uint16_t ERROR_NO_HALL_CALIBRATION = 0x0010;
  static constexpr uint16_t ERROR_OVER_CURRENT = 0x0020;
  static constexpr uint16_t ERROR_POSITION_ERROR = 0x0040;
  
  // Commands
  static constexpr uint16_t CMD_DISENGAGE = 0x0000;
  static constexpr uint16_t CMD_ENGAGE = 0x0001;
  static constexpr uint16_t CMD_ENGAGE_SMART_GRIP = 0x0002;
  static constexpr uint16_t CMD_AUTO_CALIBRATE = 0x0006;
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__MG10_DRIVER_HPP_
