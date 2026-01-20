#include "onrobot_driver/onrobot_base.hpp"

namespace onrobot_driver
{

OnRobotBase::OnRobotBase(std::shared_ptr<ModbusTCPClient> modbus_client, 
                         uint8_t device_address)
: modbus_client_(modbus_client), device_address_(device_address)
{
}

bool OnRobotBase::readProductCode(uint16_t& product_code)
{
  return readRegister(REG_PRODUCT_CODE, product_code);
}

bool OnRobotBase::readFirmwareVersion(uint8_t& major, uint8_t& minor, uint16_t& build)
{
  uint16_t version_reg, build_reg;
  
  if (!readRegister(REG_FW_VERSION_MAJOR_MINOR, version_reg)) {
    return false;
  }
  
  if (!readRegister(REG_FW_VERSION_BUILD, build_reg)) {
    return false;
  }
  
  major = (version_reg >> 8) & 0xFF;
  minor = version_reg & 0xFF;
  build = build_reg;
  
  return true;
}

bool OnRobotBase::readSerialNumber(std::string& serial_number)
{
  std::vector<uint16_t> serial_data;
  
  if (!readRegisters(REG_SERIAL_NUMBER_START, 16, serial_data)) {
    return false;
  }
  
  serial_number.clear();
  for (const auto& reg : serial_data) {
    char c1 = (reg >> 8) & 0xFF;
    char c2 = reg & 0xFF;
    
    if (c1 != 0) serial_number += c1;
    if (c2 != 0) serial_number += c2;
  }
  
  return true;
}

bool OnRobotBase::readRegister(uint16_t address, uint16_t& value)
{
  std::vector<uint16_t> values;
  if (!modbus_client_->readHoldingRegisters(device_address_, address, 1, values)) {
    return false;
  }
  value = values[0];
  return true;
}

bool OnRobotBase::readRegisters(uint16_t start_address, uint16_t quantity, 
                                std::vector<uint16_t>& values)
{
  return modbus_client_->readHoldingRegisters(device_address_, start_address, 
                                              quantity, values);
}

bool OnRobotBase::writeRegister(uint16_t address, uint16_t value)
{
  return modbus_client_->writeSingleRegister(device_address_, address, value);
}

bool OnRobotBase::writeRegisters(uint16_t start_address, const std::vector<uint16_t>& values)
{
  return modbus_client_->writeMultipleRegisters(device_address_, start_address, values);
}

} // namespace onrobot_driver
