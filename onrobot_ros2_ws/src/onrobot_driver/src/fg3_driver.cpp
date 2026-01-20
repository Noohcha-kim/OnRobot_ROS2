#include "onrobot_driver/fg3_driver.hpp"
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

FG3Driver::FG3Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address)
  : OnRobotBase(client, device_address)
  , max_diameter_(250.0f)
  , max_force_(1000.0f)  // 10*% units, 1000 = 100%
{
}

bool FG3Driver::initialize()
{
  // Read product code
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to read product code");
    return false;
  }
  
  // Determine model from product code
  // 3FG15: 0x40, 3FG25: 0x41
  switch (product_code) {
    case 0x40: // 3FG15
      max_diameter_ = 150.0f;
      max_force_ = 1000.0f;  // 10*%, 1000 = 100%
      RCLCPP_INFO(rclcpp::get_logger("fg3_driver"), "Initialized 3FG15");
      break;
    case 0x41: // 3FG25
      max_diameter_ = 250.0f;
      max_force_ = 1000.0f;  // 10*%, 1000 = 100%
      RCLCPP_INFO(rclcpp::get_logger("fg3_driver"), "Initialized 3FG25");
      break;
    default:
      RCLCPP_WARN(rclcpp::get_logger("fg3_driver"), 
                  "Unknown 3FG model: 0x%04X, using default parameters", product_code);
      max_diameter_ = 250.0f;
      max_force_ = 1000.0f;
      break;
  }
  
  return true;
}

bool FG3Driver::update()
{
  // 3FG status is read on-demand via readStatus()
  return true;
}

bool FG3Driver::grip(float diameter_mm, float force_percent, uint8_t grip_type)
{
  // Clamp values to valid range
  diameter_mm = std::max(0.0f, std::min(diameter_mm, max_diameter_));
  force_percent = std::max(0.0f, std::min(force_percent, 100.0f));
  
  // Convert diameter to device units (0.1 mm)
  uint16_t diameter_raw = static_cast<uint16_t>(diameter_mm * 10.0f);
  
  // Convert force to device units (10*%, so 100% = 1000)
  uint16_t force_raw = static_cast<uint16_t>(force_percent * 10.0f);
  
  // Write target force (10*% units)
  if (!writeRegister(REG_TARGET_FORCE, force_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to write target force");
    return false;
  }
  
  // Write target diameter
  if (!writeRegister(REG_TARGET_DIAMETER, diameter_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to write target diameter");
    return false;
  }
  
  // Write grip type (0=external, 1=internal)
  if (!writeRegister(REG_GRIP_TYPE, grip_type)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to write grip type");
    return false;
  }
  
  // Send grip command
  if (!writeRegister(REG_COMMAND, CMD_GRIP)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to send grip command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fg3_driver"), 
              "Grip command: diameter=%.1f mm, force=%.1f%%, type=%s", 
              diameter_mm, force_percent, grip_type == 0 ? "external" : "internal");
  return true;
}

bool FG3Driver::move(float diameter_mm, uint8_t grip_type)
{
  // Move without force - just call grip with 0 force
  return grip(diameter_mm, 0.0f, grip_type);
}

bool FG3Driver::flexibleGrip(float diameter_mm, float force_percent, uint8_t grip_type)
{
  // Clamp values
  diameter_mm = std::max(0.0f, std::min(diameter_mm, max_diameter_));
  force_percent = std::max(0.0f, std::min(force_percent, 100.0f));
  
  uint16_t diameter_raw = static_cast<uint16_t>(diameter_mm * 10.0f);
  uint16_t force_raw = static_cast<uint16_t>(force_percent * 10.0f);
  
  // Write parameters
  if (!writeRegister(REG_TARGET_FORCE, force_raw)) return false;
  if (!writeRegister(REG_TARGET_DIAMETER, diameter_raw)) return false;
  if (!writeRegister(REG_GRIP_TYPE, grip_type)) return false;
  
  // Send flexible grip command
  if (!writeRegister(REG_COMMAND, CMD_FLEXIBLE_GRIP)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to send flexible grip command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fg3_driver"), 
              "Flexible grip: diameter=%.1f mm, force=%.1f%%", diameter_mm, force_percent);
  return true;
}

bool FG3Driver::stop()
{
  if (!writeRegister(REG_COMMAND, CMD_STOP)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to send stop command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fg3_driver"), "Stop command sent");
  return true;
}

bool FG3Driver::readStatus(onrobot_msgs::msg::FG3Status& status)
{
  uint16_t status_reg, raw_diam, diam_offset, force;
  
  // Read status register (0x0100) - bitmask
  if (!readRegister(REG_STATUS, status_reg)) {
    RCLCPP_ERROR(rclcpp::get_logger("fg3_driver"), "Failed to read status register");
    return false;
  }
  
  // Extract status bits
  status.busy = (status_reg & STATUS_BUSY) != 0;
  status.grip_detected = (status_reg & STATUS_GRIP_DETECTED) != 0;
  status.force_grip_detected = (status_reg & STATUS_FORCE_GRIP_DETECTED) != 0;
  
  // Read raw diameter (0x0101) - 0.1mm units
  if (readRegister(REG_RAW_DIAMETER, raw_diam)) {
    status.raw_diameter = static_cast<float>(raw_diam) / 10.0f;
  }
  
  // Read diameter with offset (0x0102) - 0.1mm units
  if (readRegister(REG_DIAMETER_WITH_OFFSET, diam_offset)) {
    status.diameter_with_offset = static_cast<float>(diam_offset) / 10.0f;
  }
  
  // Read force applied (0x0103) - N units
  if (readRegister(REG_FORCE_APPLIED, force)) {
    status.force_applied = static_cast<float>(force);
  }
  
  // Read min/max diameters
  uint16_t min_diam, max_diam;
  if (readRegister(REG_MINIMUM_DIAMETER, min_diam)) {
    status.min_diameter = static_cast<float>(min_diam) / 10.0f;
  }
  if (readRegister(REG_MAXIMUM_DIAMETER, max_diam)) {
    status.max_diameter = static_cast<float>(max_diam) / 10.0f;
  }
  
  // Read additional info
  uint16_t finger_len, finger_pos, fingertip_off;
  if (readRegister(0x010E, finger_len)) {
    status.finger_length = finger_len;
  }
  if (readRegister(0x0110, finger_pos)) {
    status.finger_position = finger_pos;
  }
  if (readRegister(0x0111, fingertip_off)) {
    status.fingertip_offset = fingertip_off;
  }
  
  return true;
}

} // namespace onrobot_driver
