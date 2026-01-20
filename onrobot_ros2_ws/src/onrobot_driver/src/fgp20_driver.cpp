#include "onrobot_driver/fgp20_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

FGP20Driver::FGP20Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address)
  : OnRobotBase(client, device_address), max_width_(200.0f), max_force_(20.0f)
{
}

bool FGP20Driver::initialize()
{
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to read product code");
    return false;
  }
  
  if (product_code != 0x32) {  // 2FGP20 product code
    RCLCPP_WARN(rclcpp::get_logger("fgp20_driver"), 
                "Product code 0x%04X may not be 2FGP20 (expected 0x32)", product_code);
  }
  
  // Read max width and force from device
  uint16_t max_width_raw;
  if (readRegister(REG_MAX_WIDTH, max_width_raw)) {
    max_width_ = static_cast<float>(max_width_raw) / 10.0f;
  }
  
  uint16_t max_force_raw;
  if (readRegister(REG_MAX_FORCE, max_force_raw)) {
    max_force_ = static_cast<float>(max_force_raw) / 10.0f;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fgp20_driver"), 
              "2FGP20 initialized - Max width: %.1f mm, Max force: %.1f N", 
              max_width_, max_force_);
  
  // Initialize with vacuum off and gripper stopped
  vacuumOff();
  return stop();
}

bool FGP20Driver::update()
{
  // FGP20 driver doesn't need periodic updates
  // Status is read on demand via readStatus()
  return true;
}

bool FGP20Driver::grip(float width_mm, float force_n, float speed_percent)
{
  // Clamp values to valid range
  width_mm = std::max(0.0f, std::min(width_mm, max_width_));
  force_n = std::max(0.0f, std::min(force_n, max_force_));
  speed_percent = std::max(0.0f, std::min(speed_percent, 100.0f));
  
  // Convert to device units (0.1 mm, 0.1 N, 0.1 %)
  uint16_t width_raw = static_cast<uint16_t>(width_mm * 10.0f);
  uint16_t force_raw = static_cast<uint16_t>(force_n * 10.0f);
  uint16_t speed_raw = static_cast<uint16_t>(speed_percent * 10.0f);
  
  // Write target values
  if (!writeRegister(REG_FG_WIDTH, width_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to write target width");
    return false;
  }
  
  if (!writeRegister(REG_FG_FORCE, force_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to write target force");
    return false;
  }
  
  if (!writeRegister(REG_FG_SPEED, speed_raw)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to write speed");
    return false;
  }
  
  // Send grip command
  if (!writeRegister(REG_FG_COMMAND, CMD_GRIP_EXTERNAL)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to send grip command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fgp20_driver"), 
              "Grip command: width=%.1f mm, force=%.1f N, speed=%.1f%%", 
              width_mm, force_n, speed_percent);
  return true;
}

bool FGP20Driver::stop()
{
  if (!writeRegister(REG_FG_COMMAND, CMD_STOP)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to send stop command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fgp20_driver"), "Stop command sent");
  return true;
}

bool FGP20Driver::setVacuum(uint8_t vacuum_percent, bool ignore)
{
  // Clamp vacuum to 0-100%
  vacuum_percent = std::min(vacuum_percent, static_cast<uint8_t>(100));
  
  // Build VG command register
  // Bits 0-7: vacuum percentage
  // Bit 15: ignore flag
  uint16_t vg_command = vacuum_percent;
  if (ignore) {
    vg_command |= 0x8000;  // Set bit 15
  }
  
  if (!writeRegister(REG_VG_COMMAND, vg_command)) {
    RCLCPP_ERROR(rclcpp::get_logger("fgp20_driver"), "Failed to set vacuum command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fgp20_driver"), 
              "Vacuum set: %d%%, ignore=%s", vacuum_percent, ignore ? "true" : "false");
  return true;
}

bool FGP20Driver::vacuumOn(uint8_t vacuum_percent)
{
  return setVacuum(vacuum_percent, false);
}

bool FGP20Driver::vacuumOff()
{
  return setVacuum(0, false);
}

bool FGP20Driver::gripWithVacuum(float width_mm, float force_n, uint8_t vacuum_percent)
{
  // First set vacuum
  if (!vacuumOn(vacuum_percent)) {
    return false;
  }
  
  // Then grip
  if (!grip(width_mm, force_n)) {
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("fgp20_driver"), 
              "Grip with vacuum: width=%.1f mm, force=%.1f N, vacuum=%d%%", 
              width_mm, force_n, vacuum_percent);
  return true;
}

bool FGP20Driver::readStatus(onrobot_msgs::msg::FGStatus& status)
{
  uint16_t value;
  
  // Read status register
  if (!readRegister(REG_STATUS, value)) {
    return false;
  }
  
  status.busy = (value & STATUS_BUSY) != 0;
  status.grip_detected = (value & STATUS_GRIP_DETECTED) != 0;
  status.force_grip_detected = (value & STATUS_FORCE_GRIP) != 0;
  
  // Read external width (current position)
  if (!readRegister(REG_EXTERNAL_WIDTH, value)) {
    return false;
  }
  status.raw_diameter = static_cast<float>(value) / 10.0f;
  
  // Read current force
  if (!readRegister(REG_CURRENT_FORCE, value)) {
    return false;
  }
  status.force_applied = static_cast<float>(value) / 10.0f;
  
  // Read vacuum level
  uint16_t vacuum_raw;
  if (readRegister(REG_VG_VACUUM, vacuum_raw)) {
    // Store vacuum info in force_applied for now (could extend message)
    // Vacuum is in percentage 0-100
  }
  
  // Read min/max width
  if (readRegister(REG_MIN_WIDTH, value)) {
    status.min_diameter = static_cast<float>(value) / 10.0f;
  }
  
  if (readRegister(REG_MAX_WIDTH, value)) {
    status.max_diameter = static_cast<float>(value) / 10.0f;
  }
  
  return true;
}

} // namespace onrobot_driver
