#include "onrobot_driver/hex_driver.hpp"
#include <rclcpp/rclcpp.hpp>

namespace onrobot_driver
{

HEXDriver::HEXDriver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address)
  : OnRobotBase(client, device_address)
{
}

bool HEXDriver::initialize()
{
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read product code");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("hex_driver"), 
              "HEX-E/H Driver initialized - Product: 0x%04X", product_code);
  
  // Zero the sensor on initialization
  return zero();
}

bool HEXDriver::update()
{
  // HEX driver doesn't need periodic updates
  // Wrench is read on demand via readWrench()
  return true;
}

bool HEXDriver::zero()
{
  if (!writeRegister(REG_ZERO, CMD_ZERO)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to send zero command");
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("hex_driver"), "Sensor zeroed");
  return true;
}

bool HEXDriver::readWrench(onrobot_msgs::msg::HEXWrench& wrench)
{
  uint16_t value;
  
  // Read force X (0.1 N units)
  if (!readRegister(REG_FX, value)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read FX");
    return false;
  }
  wrench.force.x = static_cast<float>(toInt16(value)) / 10.0f;
  
  // Read force Y
  if (!readRegister(REG_FY, value)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read FY");
    return false;
  }
  wrench.force.y = static_cast<float>(toInt16(value)) / 10.0f;
  
  // Read force Z
  if (!readRegister(REG_FZ, value)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read FZ");
    return false;
  }
  wrench.force.z = static_cast<float>(toInt16(value)) / 10.0f;
  
  // Read torque X (0.001 Nm units)
  if (!readRegister(REG_TX, value)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read TX");
    return false;
  }
  wrench.torque.x = static_cast<float>(toInt16(value)) / 1000.0f;
  
  // Read torque Y
  if (!readRegister(REG_TY, value)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read TY");
    return false;
  }
  wrench.torque.y = static_cast<float>(toInt16(value)) / 1000.0f;
  
  // Read torque Z
  if (!readRegister(REG_TZ, value)) {
    RCLCPP_ERROR(rclcpp::get_logger("hex_driver"), "Failed to read TZ");
    return false;
  }
  wrench.torque.z = static_cast<float>(toInt16(value)) / 1000.0f;
  
  // Read status register (0x0101)
  // Status reads 0x0000 when no error
  if (readRegister(REG_STATUS, value)) {
    wrench.sensor_error = (value != 0x0000);
  } else {
    wrench.sensor_error = true;  // If can't read status, assume error
  }
  
  // HEX doesn't have a "zeroed" flag, assume always valid
  wrench.zeroed = true;
  
  // Set timestamp
  wrench.header.stamp = rclcpp::Clock().now();
  wrench.header.frame_id = "hex_sensor";
  
  return true;
}

int16_t HEXDriver::toInt16(uint16_t value)
{
  // Convert unsigned to signed (two's complement)
  if (value & 0x8000) {
    return static_cast<int16_t>(value) - 0x10000;
  }
  return static_cast<int16_t>(value);
}

} // namespace onrobot_driver
