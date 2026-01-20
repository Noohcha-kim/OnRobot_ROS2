#ifndef ONROBOT_DRIVER__RG_DRIVER_HPP_
#define ONROBOT_DRIVER__RG_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/rg_status.hpp"
#include "onrobot_msgs/msg/rg_command.hpp"
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

// RG2/RG6 Register addresses
constexpr uint16_t RG_REG_TARGET_FORCE = 0x0000;
constexpr uint16_t RG_REG_TARGET_WIDTH = 0x0001;
constexpr uint16_t RG_REG_CONTROL = 0x0002;
constexpr uint16_t RG_REG_FINGERTIP_OFFSET = 0x0102;
constexpr uint16_t RG_REG_ACTUAL_DEPTH = 0x0107;
constexpr uint16_t RG_REG_ACTUAL_RELATIVE_DEPTH = 0x0108;
constexpr uint16_t RG_REG_ACTUAL_WIDTH = 0x010B;
constexpr uint16_t RG_REG_STATUS = 0x010C;
constexpr uint16_t RG_REG_ACTUAL_WIDTH_WITH_OFFSET = 0x0113;
constexpr uint16_t RG_REG_SET_FINGERTIP_OFFSET = 0x0407;

// Control commands
constexpr uint16_t RG_CMD_GRIP = 0x0001;
constexpr uint16_t RG_CMD_STOP = 0x0008;
constexpr uint16_t RG_CMD_GRIP_WITH_OFFSET = 0x0010;

// Status bits
constexpr uint16_t RG_STATUS_BUSY = 0x0001;
constexpr uint16_t RG_STATUS_GRIP_DETECTED = 0x0002;
constexpr uint16_t RG_STATUS_S1_PUSHED = 0x0004;
constexpr uint16_t RG_STATUS_S1_TRIGGERED = 0x0008;
constexpr uint16_t RG_STATUS_S2_PUSHED = 0x0010;
constexpr uint16_t RG_STATUS_S2_TRIGGERED = 0x0020;
constexpr uint16_t RG_STATUS_SAFETY_ERROR = 0x0040;

class RGDriver : public OnRobotBase
{
public:
  RGDriver(std::shared_ptr<ModbusTCPClient> modbus_client, uint8_t device_address);
  ~RGDriver() override = default;

  // Gripper control
  bool grip(float target_width_mm, float target_force_n);
  bool gripWithOffset(float target_width_mm, float target_force_n);
  bool stop();
  
  // Configuration
  bool setFingertipOffset(float offset_mm);
  
  // Status
  bool readStatus(onrobot_msgs::msg::RGStatus& status);
  
  // Inherited from OnRobotBase
  bool initialize() override;
  bool update() override;

private:
  onrobot_msgs::msg::RGStatus current_status_;
  
  // Helper functions
  int16_t mmToTenthMm(float mm);
  float tenthMmToMm(int16_t tenth_mm);
  int16_t nToTenthN(float n);
  float tenthNToN(int16_t tenth_n);
  
  bool updateStatus();
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__RG_DRIVER_HPP_
