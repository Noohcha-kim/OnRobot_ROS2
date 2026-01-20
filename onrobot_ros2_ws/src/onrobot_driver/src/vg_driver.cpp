#include "onrobot_driver/vg_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

VGDriver::VGDriver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address)
  : OnRobotBase(client, device_address), num_channels_(2)
{
}

bool VGDriver::initialize()
{
  // Read product code to determine number of channels
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), "Failed to read product code");
    return false;
  }
  
  // VG10/VGC10: 0x10/0x11 (2 channels)
  // VGP20: 0x12 (4 channels)  
  // VGP30: 0x13 (2 channels)
  if (product_code == 0x12) {
    num_channels_ = 4;
  } else {
    num_channels_ = 2;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("vg_driver"), 
              "VG Driver initialized - Product: 0x%04X, Channels: %d", 
              product_code, num_channels_);
  
  // Stop all channels initially
  return stopAll();
}

bool VGDriver::update()
{
  // VG driver doesn't need periodic updates
  // Status is read on demand via readStatus()
  return true;
}

bool VGDriver::setControl(const std::vector<uint8_t>& control_modes,
                          const std::vector<uint8_t>& vacuum_levels)
{
  if (control_modes.size() != num_channels_ || vacuum_levels.size() != num_channels_) {
    RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                 "Invalid vector size. Expected %d channels", num_channels_);
    return false;
  }
  
  // Write control for each channel
  // Register format: [Control mode (bits 15-8)] [Target vacuum (bits 7-0)]
  for (uint8_t i = 0; i < num_channels_; i++) {
    uint8_t control_mode = control_modes[i];
    uint8_t vacuum = std::min(vacuum_levels[i], static_cast<uint8_t>(80));
    
    // Combine: (control_mode << 8) | vacuum
    uint16_t control_value = (static_cast<uint16_t>(control_mode) << 8) | vacuum;
    
    if (!writeRegister(REG_CONTROL_A + i, control_value)) {
      RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                   "Failed to write control for channel %d", i);
      return false;
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("vg_driver"), 
                 "Channel %d: mode=%d, vacuum=%d%%, value=0x%04X", 
                 i, control_mode, vacuum, control_value);
  }
  
  return true;
}

// Overload for float32 (from ROS message)
bool VGDriver::setControlFloat(const std::vector<uint8_t>& control_modes,
                                const std::vector<float>& vacuum_levels)
{
  if (control_modes.size() != num_channels_ || vacuum_levels.size() != num_channels_) {
    RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                 "Invalid vector size. Expected %d channels", num_channels_);
    return false;
  }
  
  // Write control for each channel
  // Register format: [Control mode (bits 15-8)] [Target vacuum (bits 7-0)]
  for (uint8_t i = 0; i < num_channels_; i++) {
    uint8_t control_mode = control_modes[i];
    uint8_t vacuum = std::min(static_cast<uint8_t>(vacuum_levels[i]), static_cast<uint8_t>(80));
    
    // Combine: (control_mode << 8) | vacuum
    uint16_t control_value = (static_cast<uint16_t>(control_mode) << 8) | vacuum;
    
    if (!writeRegister(REG_CONTROL_A + i, control_value)) {
      RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                   "Failed to write control for channel %d", i);
      return false;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("vg_driver"), 
                "Channel %d: mode=%d, vacuum=%.0f%%, value=0x%04X", 
                i, control_mode, vacuum_levels[i], control_value);
  }
  
  return true;
}

bool VGDriver::setChannelControl(uint8_t channel, uint8_t control_mode, uint8_t vacuum_level)
{
  if (channel >= num_channels_) {
    RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), "Invalid channel: %d", channel);
    return false;
  }
  
  // Clamp vacuum to 0-80%
  uint8_t vacuum = std::min(vacuum_level, static_cast<uint8_t>(80));
  
  // Combine: (control_mode << 8) | vacuum
  uint16_t control_value = (static_cast<uint16_t>(control_mode) << 8) | vacuum;
  
  if (!writeRegister(REG_CONTROL_A + channel, control_value)) {
    RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                 "Failed to write control for channel %d", channel);
    return false;
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("vg_driver"), 
               "Channel %d: mode=%d, vacuum=%d%%, value=0x%04X", 
               channel, control_mode, vacuum, control_value);
  
  return true;
}

bool VGDriver::stopAll()
{
  // Set all channels to RELEASE (0)
  // Register format: [Control mode (bits 15-8)] [Target vacuum (bits 7-0)]
  // RELEASE = 0x0000 (mode=0, vacuum=0)
  for (uint8_t i = 0; i < num_channels_; i++) {
    if (!writeRegister(REG_CONTROL_A + i, 0x0000)) {
      RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                   "Failed to stop channel %d", i);
      return false;
    }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("vg_driver"), "All channels stopped");
  return true;
}

bool VGDriver::readStatus(onrobot_msgs::msg::VGStatus& status)
{
  status.num_channels = num_channels_;
  status.vacuum_level.resize(num_channels_);
  status.grip_status.resize(num_channels_, 0);  // VG10/VGC10 doesn't have grip status
  status.release_status.resize(num_channels_, 0);
  
  // VG10/VGC10 doesn't have busy/error registers
  status.busy = false;
  status.psu_error = false;
  status.qc_error = false;
  
  // Read actual vacuum levels for each channel
  // Vacuum is in 1/1000 of relative vacuum (0-1000)
  for (uint8_t i = 0; i < num_channels_; i++) {
    uint16_t vacuum_raw;
    if (!readRegister(REG_VACUUM_LEVEL_A + i, vacuum_raw)) {
      RCLCPP_ERROR(rclcpp::get_logger("vg_driver"), 
                   "Failed to read vacuum level for channel %d", i);
      return false;
    }
    
    // Convert from 1/1000 relative vacuum to percentage
    // 1000 = 100% vacuum
    status.vacuum_level[i] = static_cast<float>(vacuum_raw) / 10.0f;
    
    // Simple grip detection: if vacuum > 10%, assume gripping
    if (vacuum_raw > 100) {
      status.grip_status[i] = 1;  // GRIP_DETECTED
    } else {
      status.grip_status[i] = 0;  // GRIP_NOT_GRIPPED
    }
  }
  
  return true;
}

// setRequireGrip and setIgnoreChannel are not supported by VG10/VGC10 hardware
// These functions are kept for API compatibility but do nothing
bool VGDriver::setRequireGrip(const std::vector<bool>& /*require_grip*/)
{
  // VG10/VGC10 doesn't have require_grip register
  // This is a no-op for compatibility
  return true;
}

bool VGDriver::setIgnoreChannel(const std::vector<bool>& /*ignore_channel*/)
{
  // VG10/VGC10 doesn't have ignore_channel register  
  // This is a no-op for compatibility
  return true;
}

} // namespace onrobot_driver
