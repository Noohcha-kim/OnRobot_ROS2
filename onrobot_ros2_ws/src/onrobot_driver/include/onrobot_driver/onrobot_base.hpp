#ifndef ONROBOT_DRIVER__ONROBOT_BASE_HPP_
#define ONROBOT_DRIVER__ONROBOT_BASE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "onrobot_driver/modbus_tcp_client.hpp"

namespace onrobot_driver
{

// Common register addresses (shared by all devices)
constexpr uint16_t REG_PRODUCT_CODE = 0x0600;
constexpr uint16_t REG_FW_VERSION_MAJOR_MINOR = 0x0604;
constexpr uint16_t REG_FW_VERSION_BUILD = 0x0605;
constexpr uint16_t REG_SERIAL_NUMBER_START = 0x0609;
constexpr uint16_t REG_SERIAL_NUMBER_END = 0x0618;

// Product codes
constexpr uint16_t PRODUCT_VG10 = 0x10;
constexpr uint16_t PRODUCT_VGC10 = 0x11;
constexpr uint16_t PRODUCT_VGP20 = 0x18;
constexpr uint16_t PRODUCT_RG2 = 0x20;
constexpr uint16_t PRODUCT_RG6 = 0x21;
constexpr uint16_t PRODUCT_RG2_FT = 0x22;
constexpr uint16_t PRODUCT_SG = 0x50;
constexpr uint16_t PRODUCT_3FG15 = 0x70;
constexpr uint16_t PRODUCT_3FG25 = 0x71;
constexpr uint16_t PRODUCT_SCREWDRIVER = 0x80;
constexpr uint16_t PRODUCT_LIFT100_V1 = 0x100;
constexpr uint16_t PRODUCT_LIFT100_V2 = 0x101;
constexpr uint16_t PRODUCT_MG10 = 0xA0;
constexpr uint16_t PRODUCT_SANDER = 0xB0;
constexpr uint16_t PRODUCT_2FG7 = 0xC0;
constexpr uint16_t PRODUCT_2FG14 = 0xC1;
constexpr uint16_t PRODUCT_2FGP20 = 0xF0;

class OnRobotBase
{
public:
  OnRobotBase(std::shared_ptr<ModbusTCPClient> modbus_client, 
              uint8_t device_address);
  virtual ~OnRobotBase() = default;

  // Common functions
  bool readProductCode(uint16_t& product_code);
  bool readFirmwareVersion(uint8_t& major, uint8_t& minor, uint16_t& build);
  bool readSerialNumber(std::string& serial_number);
  
  // Device control
  virtual bool initialize() = 0;
  virtual bool update() = 0;

protected:
  std::shared_ptr<ModbusTCPClient> modbus_client_;
  uint8_t device_address_;
  
  // Helper functions
  bool readRegister(uint16_t address, uint16_t& value);
  bool readRegisters(uint16_t start_address, uint16_t quantity, 
                    std::vector<uint16_t>& values);
  bool writeRegister(uint16_t address, uint16_t value);
  bool writeRegisters(uint16_t start_address, const std::vector<uint16_t>& values);
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__ONROBOT_BASE_HPP_
