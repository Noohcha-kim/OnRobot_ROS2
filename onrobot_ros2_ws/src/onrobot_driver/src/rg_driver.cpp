#include "onrobot_driver/rg_driver.hpp"
#include <cmath>

namespace onrobot_driver
{

RGDriver::RGDriver(std::shared_ptr<ModbusTCPClient> modbus_client, uint8_t device_address)
: OnRobotBase(modbus_client, device_address)
{
}

bool RGDriver::initialize()
{
  // Read product code to verify correct gripper type
  uint16_t product_code;
  if (!readProductCode(product_code)) {
    return false;
  }
  
  if (product_code != PRODUCT_RG2 && product_code != PRODUCT_RG6) {
    return false;
  }
  
  current_status_.product_code = product_code;
  
  // Read serial number
  std::string serial;
  if (readSerialNumber(serial)) {
    current_status_.serial_number = serial;
  }
  
  return updateStatus();
}

bool RGDriver::update()
{
  return updateStatus();
}

bool RGDriver::grip(float target_width_mm, float target_force_n)
{
  // Write target force
  int16_t force_tenth_n = nToTenthN(target_force_n);
  if (!writeRegister(RG_REG_TARGET_FORCE, static_cast<uint16_t>(force_tenth_n))) {
    return false;
  }
  
  // Write target width
  int16_t width_tenth_mm = mmToTenthMm(target_width_mm);
  if (!writeRegister(RG_REG_TARGET_WIDTH, static_cast<uint16_t>(width_tenth_mm))) {
    return false;
  }
  
  // Send grip command
  return writeRegister(RG_REG_CONTROL, RG_CMD_GRIP);
}

bool RGDriver::gripWithOffset(float target_width_mm, float target_force_n)
{
  // Write target force
  int16_t force_tenth_n = nToTenthN(target_force_n);
  if (!writeRegister(RG_REG_TARGET_FORCE, static_cast<uint16_t>(force_tenth_n))) {
    return false;
  }
  
  // Write target width
  int16_t width_tenth_mm = mmToTenthMm(target_width_mm);
  if (!writeRegister(RG_REG_TARGET_WIDTH, static_cast<uint16_t>(width_tenth_mm))) {
    return false;
  }
  
  // Send grip with offset command
  return writeRegister(RG_REG_CONTROL, RG_CMD_GRIP_WITH_OFFSET);
}

bool RGDriver::stop()
{
  return writeRegister(RG_REG_CONTROL, RG_CMD_STOP);
}

bool RGDriver::setFingertipOffset(float offset_mm)
{
  int16_t offset_tenth_mm = mmToTenthMm(offset_mm);
  return writeRegister(RG_REG_SET_FINGERTIP_OFFSET, static_cast<uint16_t>(offset_tenth_mm));
}

bool RGDriver::readStatus(onrobot_msgs::msg::RGStatus& status)
{
  if (!updateStatus()) {
    return false;
  }
  status = current_status_;
  return true;
}

bool RGDriver::updateStatus()
{
  // Read status register
  uint16_t status_reg;
  if (!readRegister(RG_REG_STATUS, status_reg)) {
    return false;
  }
  
  // Parse status flags
  current_status_.busy = (status_reg & RG_STATUS_BUSY) != 0;
  current_status_.grip_detected = (status_reg & RG_STATUS_GRIP_DETECTED) != 0;
  current_status_.s1_pushed = (status_reg & RG_STATUS_S1_PUSHED) != 0;
  current_status_.s1_triggered = (status_reg & RG_STATUS_S1_TRIGGERED) != 0;
  current_status_.s2_pushed = (status_reg & RG_STATUS_S2_PUSHED) != 0;
  current_status_.s2_triggered = (status_reg & RG_STATUS_S2_TRIGGERED) != 0;
  current_status_.safety_error = (status_reg & RG_STATUS_SAFETY_ERROR) != 0;
  
  // Read actual width
  uint16_t width_reg;
  if (readRegister(RG_REG_ACTUAL_WIDTH, width_reg)) {
    current_status_.actual_width = tenthMmToMm(static_cast<int16_t>(width_reg));
  }
  
  // Read actual width with offset
  uint16_t width_offset_reg;
  if (readRegister(RG_REG_ACTUAL_WIDTH_WITH_OFFSET, width_offset_reg)) {
    current_status_.actual_width_offset = tenthMmToMm(static_cast<int16_t>(width_offset_reg));
  }
  
  // Read actual depth
  uint16_t depth_reg;
  if (readRegister(RG_REG_ACTUAL_DEPTH, depth_reg)) {
    current_status_.actual_depth = tenthMmToMm(static_cast<int16_t>(depth_reg));
  }
  
  // Read actual relative depth
  uint16_t rel_depth_reg;
  if (readRegister(RG_REG_ACTUAL_RELATIVE_DEPTH, rel_depth_reg)) {
    current_status_.actual_relative_depth = tenthMmToMm(static_cast<int16_t>(rel_depth_reg));
  }
  
  // Read fingertip offset
  uint16_t offset_reg;
  if (readRegister(RG_REG_FINGERTIP_OFFSET, offset_reg)) {
    current_status_.fingertip_offset = tenthMmToMm(static_cast<int16_t>(offset_reg));
  }
  
  return true;
}

int16_t RGDriver::mmToTenthMm(float mm)
{
  return static_cast<int16_t>(std::round(mm * 10.0f));
}

float RGDriver::tenthMmToMm(int16_t tenth_mm)
{
  return static_cast<float>(tenth_mm) / 10.0f;
}

int16_t RGDriver::nToTenthN(float n)
{
  return static_cast<int16_t>(std::round(n * 10.0f));
}

float RGDriver::tenthNToN(int16_t tenth_n)
{
  return static_cast<float>(tenth_n) / 10.0f;
}

} // namespace onrobot_driver
