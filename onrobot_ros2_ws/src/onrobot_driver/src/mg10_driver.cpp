#include "onrobot_driver/mg10_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

MG10Driver::MG10Driver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address)
  : OnRobotBase(client, device_address)
{
}

bool MG10Driver::initialize()
{
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to read product code");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("mg10_driver"), 
              "MG10 Driver initialized - Product: 0x%04X", product_code);
  
  // Start with magnet disengaged
  return disengage();
}

bool MG10Driver::update()
{
  // MG10 driver doesn't need periodic updates
  // Status is read on demand via readStatus()
  return true;
}

bool MG10Driver::disengage()
{
  if (!writeRegister(REG_COMMAND, CMD_DISENGAGE)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to send disengage command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("mg10_driver"), "Magnet disengaged");
  return true;
}

bool MG10Driver::engage(uint8_t strength)
{
  // Clamp strength to 0-100%
  strength = std::min(strength, static_cast<uint8_t>(100));
  
  // Set target strength
  if (!writeRegister(REG_TARGET_STRENGTH, strength)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to set target strength");
    return false;
  }
  
  // Send engage command
  if (!writeRegister(REG_COMMAND, CMD_ENGAGE)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to send engage command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("mg10_driver"), "Magnet engaged at %d%% strength", strength);
  return true;
}

bool MG10Driver::engageSmartGrip(uint8_t strength)
{
  // Clamp strength to 0-100%
  strength = std::min(strength, static_cast<uint8_t>(100));
  
  // Set target strength
  if (!writeRegister(REG_TARGET_STRENGTH, strength)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to set target strength");
    return false;
  }
  
  // Send smart grip command
  if (!writeRegister(REG_COMMAND, CMD_ENGAGE_SMART_GRIP)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to send smart grip command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("mg10_driver"), "Smart grip engaged at %d%% strength", strength);
  return true;
}

bool MG10Driver::autoCalibrate()
{
  if (!writeRegister(REG_COMMAND, CMD_AUTO_CALIBRATE)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to send auto-calibrate command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("mg10_driver"), "Auto-calibration started");
  return true;
}

bool MG10Driver::readStatus(onrobot_msgs::msg::MG10Status& status)
{
  uint16_t status_reg, error_reg, strength;
  
  // Read status register (0x0100) - bitmask
  if (!readRegister(REG_STATUS, status_reg)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to read status register");
    return false;
  }
  
  // Extract status bits
  status.part_gripped = (status_reg & STATUS_PART_GRIPPED) != 0;
  status.near_part = (status_reg & STATUS_NEAR_PART) != 0;
  status.busy = (status_reg & STATUS_BUSY) != 0;
  
  // Read error code register (0x0101) - bitmask
  if (!readRegister(REG_ERROR_CODE, error_reg)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to read error register");
    return false;
  }
  
  // Extract error bits
  status.over_heating = (error_reg & ERROR_OVER_HEATING) != 0;
  status.no_hall_calibration = (error_reg & ERROR_NO_HALL_CALIBRATION) != 0;
  
  // Read magnet strength (0x0102) - value 0-100%
  if (!readRegister(REG_MAGNET_STRENGTH, strength)) {
    RCLCPP_ERROR(rclcpp::get_logger("mg10_driver"), "Failed to read magnet strength");
    return false;
  }
  status.magnet_strength = static_cast<float>(strength);
  
  return true;
}

} // namespace onrobot_driver
