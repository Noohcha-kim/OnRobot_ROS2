#include "onrobot_driver/modbus_tcp_client.hpp"
#include <cstring>
#include <iostream>

namespace onrobot_driver
{

ModbusTCPClient::ModbusTCPClient(const std::string& ip_address, int port)
: ip_address_(ip_address), port_(port), socket_fd_(-1), connected_(false), transaction_id_(0)
{
}

ModbusTCPClient::~ModbusTCPClient()
{
  disconnect();
}

bool ModbusTCPClient::connect()
{
  if (connected_) {
    return true;
  }

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    std::cerr << "Failed to create socket" << std::endl;
    return false;
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);

  if (inet_pton(AF_INET, ip_address_.c_str(), &server_addr.sin_addr) <= 0) {
    std::cerr << "Invalid IP address" << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  if (::connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    std::cerr << "Connection failed" << std::endl;
    close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  connected_ = true;
  return true;
}

void ModbusTCPClient::disconnect()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
}

bool ModbusTCPClient::isConnected() const
{
  return connected_;
}

bool ModbusTCPClient::readHoldingRegisters(uint8_t device_address, uint16_t start_address,
                                           uint16_t quantity, std::vector<uint16_t>& data)
{
  if (!connected_) {
    return false;
  }

  // Build Modbus TCP request
  std::vector<uint8_t> request(12);
  
  // MBAP Header
  transaction_id_++;
  request[0] = (transaction_id_ >> 8) & 0xFF;
  request[1] = transaction_id_ & 0xFF;
  request[2] = 0x00; // Protocol ID
  request[3] = 0x00;
  request[4] = 0x00; // Length
  request[5] = 0x06;
  
  // PDU
  request[6] = device_address;
  request[7] = 0x03; // Function code: Read Holding Registers
  request[8] = (start_address >> 8) & 0xFF;
  request[9] = start_address & 0xFF;
  request[10] = (quantity >> 8) & 0xFF;
  request[11] = quantity & 0xFF;

  if (!sendRequest(request)) {
    return false;
  }

  // Receive response
  std::vector<uint8_t> response;
  size_t expected_length = 9 + (quantity * 2);
  if (!receiveResponse(response, expected_length)) {
    return false;
  }

  // Parse response
  if (response.size() < expected_length) {
    return false;
  }

  if (response[7] != 0x03) {
    std::cerr << "Invalid function code in response" << std::endl;
    return false;
  }

  uint8_t byte_count = response[8];
  if (byte_count != quantity * 2) {
    return false;
  }

  data.resize(quantity);
  for (size_t i = 0; i < quantity; i++) {
    data[i] = (response[9 + i * 2] << 8) | response[10 + i * 2];
  }

  return true;
}

bool ModbusTCPClient::writeSingleRegister(uint8_t device_address, uint16_t address, uint16_t value)
{
  if (!connected_) {
    return false;
  }

  std::vector<uint8_t> request(12);
  
  // MBAP Header
  transaction_id_++;
  request[0] = (transaction_id_ >> 8) & 0xFF;
  request[1] = transaction_id_ & 0xFF;
  request[2] = 0x00;
  request[3] = 0x00;
  request[4] = 0x00;
  request[5] = 0x06;
  
  // PDU
  request[6] = device_address;
  request[7] = 0x06; // Function code: Write Single Register
  request[8] = (address >> 8) & 0xFF;
  request[9] = address & 0xFF;
  request[10] = (value >> 8) & 0xFF;
  request[11] = value & 0xFF;

  if (!sendRequest(request)) {
    return false;
  }

  std::vector<uint8_t> response;
  if (!receiveResponse(response, 12)) {
    return false;
  }

  return (response[7] == 0x06);
}

bool ModbusTCPClient::writeMultipleRegisters(uint8_t device_address, uint16_t start_address,
                                             const std::vector<uint16_t>& values)
{
  if (!connected_ || values.empty()) {
    return false;
  }

  uint16_t quantity = values.size();
  uint8_t byte_count = quantity * 2;

  std::vector<uint8_t> request(13 + byte_count);
  
  // MBAP Header
  transaction_id_++;
  request[0] = (transaction_id_ >> 8) & 0xFF;
  request[1] = transaction_id_ & 0xFF;
  request[2] = 0x00;
  request[3] = 0x00;
  uint16_t length = 7 + byte_count;
  request[4] = (length >> 8) & 0xFF;
  request[5] = length & 0xFF;
  
  // PDU
  request[6] = device_address;
  request[7] = 0x10; // Function code: Write Multiple Registers
  request[8] = (start_address >> 8) & 0xFF;
  request[9] = start_address & 0xFF;
  request[10] = (quantity >> 8) & 0xFF;
  request[11] = quantity & 0xFF;
  request[12] = byte_count;
  
  // Register values
  for (size_t i = 0; i < values.size(); i++) {
    request[13 + i * 2] = (values[i] >> 8) & 0xFF;
    request[14 + i * 2] = values[i] & 0xFF;
  }

  if (!sendRequest(request)) {
    return false;
  }

  std::vector<uint8_t> response;
  if (!receiveResponse(response, 12)) {
    return false;
  }

  return (response[7] == 0x10);
}

bool ModbusTCPClient::sendRequest(const std::vector<uint8_t>& request)
{
  ssize_t sent = send(socket_fd_, request.data(), request.size(), 0);
  if (sent < 0 || static_cast<size_t>(sent) != request.size()) {
    std::cerr << "Failed to send request" << std::endl;
    return false;
  }
  return true;
}

bool ModbusTCPClient::receiveResponse(std::vector<uint8_t>& response, size_t expected_length)
{
  response.resize(expected_length);
  ssize_t received = recv(socket_fd_, response.data(), expected_length, 0);
  
  if (received < 0) {
    std::cerr << "Failed to receive response" << std::endl;
    return false;
  }
  
  response.resize(received);
  return true;
}

} // namespace onrobot_driver
