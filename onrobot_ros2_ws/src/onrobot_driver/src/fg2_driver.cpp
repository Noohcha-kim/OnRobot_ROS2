#include "onrobot_driver/fg2_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

FG2Driver::FG2Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address)
  : OnRobotBase(client, device_address), max_width_(0), max_force_(0)
{
}

bool FG2Driver::initialize()
{
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to read product code");
    return false;
  }
  
  // Determine model from product code
  // 2FG7: 0x30, 2FG14: 0x31
  // Note: 2FGP20 (0x32) uses FGP20Driver, not FG2Driver
  switch (product_code) {
    case 0x30: // 2FG7
      max_width_ = 70.0f;
      max_force_ = 20.0f;
      RCLCPP_INFO(rclcpp::get_logger("fg2_driver"), "Initialized 2FG7");
      break;
    case 0x31: // 2FG14
      max_width_ = 140.0f;
      max_force_ = 40.0f;
      RCLCPP_INFO(rclcpp::get_logger("fg2_driver"), "Initialized 2FG14");
      break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("fg2_driver"), 
                  "Unknown 2FG model: 0x%04X, using default parameters", product_code);
      max_width_ = 140.0f;
      max_force_ = 40.0f;
      break;
  }
  
  return stop();
}

bool FG2Driver::update()
{
  // FG2 driver doesn't need periodic updates
  // Status is read on demand via readStatus()
  return true;
}

bool FG2Driver::grip(float width_mm, float force_n)
{
  // Clamp values to valid range
  width_mm = std::max(0.0f, std::min(width_mm, max_width_));
  force_n = std::max(0.0f, std::min(force_n, max_force_));
  
  // Convert width to device units (0.1 mm)
  uint16_t width_raw = static_cast<uint16_t>(width_mm * 10.0f);
  
  // Force is in N (not 0.1N for 2FG!)
  uint16_t force_raw = static_cast<uint16_t>(force_n);
  
  // Write target width
  if (!writeRegister(REG_TARGET_WIDTH, width_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to write target width");
    return false;
  }
  
  // Write target force
  if (!writeRegister(REG_TARGET_FORCE, force_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to write target force");
    return false;
  }
  
  // Write speed (default 100%)
  if (!writeRegister(REG_TARGET_SPEED, 100)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to write target speed");
    return false;
  }
  
  // Send external grip command
  if (!writeRegister(REG_COMMAND, CMD_GRIP_EXTERNAL)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to send grip command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fg2_driver"), 
              "Grip command: width=%.1f mm, force=%.1f N", width_mm, force_n);
  return true;
}

bool FG2Driver::stop()
{
  if (!writeRegister(REG_COMMAND, CMD_STOP)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to send stop command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fg2_driver"), "Stop command sent");
  return true;
}

bool FG2Driver::readStatus(onrobot_msgs::msg::FGStatus& status)
{
  uint16_t status_reg, width, force;
  
  // Read status register (0x0100) - bitmask
  if (!readRegister(REG_STATUS, status_reg)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to read status register");
    return false;
  }
  
  // Extract status bits
  status.busy = (status_reg & STATUS_BUSY) != 0;
  status.grip_detected = (status_reg & STATUS_GRIP_DETECTED) != 0;
  
  // Read external width (0x0101) - 0.1mm units
  if (!readRegister(REG_EXTERNAL_WIDTH, width)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to read external width");
    return false;
  }
  status.raw_diameter = static_cast<float>(width) / 10.0f;  // Convert to mm
  
  // Read current force (0x0107) - N units
  if (!readRegister(REG_FORCE, force)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg2_driver"), "Failed to read force");
    return false;
  }
  status.force_applied = static_cast<float>(force);  // Already in N
  
  // Read min/max widths
  uint16_t min_ext, max_ext;
  if (readRegister(REG_MIN_EXTERNAL_WIDTH, min_ext)) {
    status.min_diameter = static_cast<float>(min_ext) / 10.0f;
  }
  if (readRegister(REG_MAX_EXTERNAL_WIDTH, max_ext)) {
    status.max_diameter = static_cast<float>(max_ext) / 10.0f;
  }
  
  status.force_grip_detected = status.grip_detected;  // Same as grip_detected for 2FG
  
  return true;
}

} // namespace onrobot_driver
