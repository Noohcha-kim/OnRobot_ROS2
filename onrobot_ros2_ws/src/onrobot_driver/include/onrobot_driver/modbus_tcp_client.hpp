#ifndef ONROBOT_DRIVER__MODBUS_TCP_CLIENT_HPP_
#define ONROBOT_DRIVER__MODBUS_TCP_CLIENT_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace onrobot_driver
{

class ModbusTCPClient
{
public:
  explicit ModbusTCPClient(const std::string& ip_address, int port = 502);
  ~ModbusTCPClient();

  // Connection management
  bool connect();
  void disconnect();
  bool isConnected() const;

  // Modbus functions
  bool readHoldingRegisters(uint8_t device_address, uint16_t start_address, 
                           uint16_t quantity, std::vector<uint16_t>& data);
  
  bool writeSingleRegister(uint8_t device_address, uint16_t address, uint16_t value);
  
  bool writeMultipleRegisters(uint8_t device_address, uint16_t start_address,
                             const std::vector<uint16_t>& values);
  
  bool readWriteMultipleRegisters(uint8_t device_address,
                                 uint16_t read_address, uint16_t read_quantity,
                                 uint16_t write_address, const std::vector<uint16_t>& write_values,
                                 std::vector<uint16_t>& read_data);

private:
  std::string ip_address_;
  int port_;
  int socket_fd_;
  bool connected_;
  uint16_t transaction_id_;

  // Helper functions
  bool sendRequest(const std::vector<uint8_t>& request);
  bool receiveResponse(std::vector<uint8_t>& response, size_t expected_length);
  uint16_t calculateCRC(const std::vector<uint8_t>& data);
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__MODBUS_TCP_CLIENT_HPP_
