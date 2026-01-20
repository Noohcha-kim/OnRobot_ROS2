#ifndef ONROBOT_DRIVER__VG_DRIVER_HPP_
#define ONROBOT_DRIVER__VG_DRIVER_HPP_

#include "onrobot_driver/onrobot_base.hpp"
#include "onrobot_msgs/msg/vg_status.hpp"
#include "onrobot_msgs/msg/vg_command.hpp"
#include <vector>

namespace onrobot_driver
{

class VGDriver : public OnRobotBase
{
public:
  VGDriver(std::shared_ptr<ModbusTCPClient> client, uint8_t device_address);
  ~VGDriver() = default;

  bool initialize() override;
  bool update() override;
  
  // VG control commands
  bool setControl(const std::vector<uint8_t>& control_modes,
                  const std::vector<uint8_t>& vacuum_levels);
  bool setControlFloat(const std::vector<uint8_t>& control_modes,
                       const std::vector<float>& vacuum_levels);
  bool setChannelControl(uint8_t channel, uint8_t control_mode, uint8_t vacuum_level);
  bool stopAll();
  
  // Status reading
  bool readStatus(onrobot_msgs::msg::VGStatus& status);
  
  // Channel configuration
  bool setRequireGrip(const std::vector<bool>& require_grip);
  bool setIgnoreChannel(const std::vector<bool>& ignore_channel);
  
  uint8_t getNumChannels() const { return num_channels_; }

private:
  // VG10/VGC10 register addresses (from Connectivity Guide v1.22)
  // Control registers contain both mode (bits 15-8) and vacuum (bits 7-0)
  static constexpr uint16_t REG_CONTROL_A = 0x0000;  // Channel A Control
  static constexpr uint16_t REG_CONTROL_B = 0x0001;  // Channel B Control
  static constexpr uint16_t REG_CONTROL_C = 0x0002;  // Channel C Control (VGP20 only)
  static constexpr uint16_t REG_CONTROL_D = 0x0003;  // Channel D Control (VGP20 only)
  
  static constexpr uint16_t REG_CURRENT_LIMIT = 0x0002;  // Current limit in mA
  
  // Status registers
  static constexpr uint16_t REG_VACUUM_LEVEL_A = 0x0102;  // Actual vacuum A (1/1000 relative)
  static constexpr uint16_t REG_VACUUM_LEVEL_B = 0x0103;  // Actual vacuum B (1/1000 relative)
  static constexpr uint16_t REG_VACUUM_LEVEL_C = 0x0104;  // Actual vacuum C (VGP20)
  static constexpr uint16_t REG_VACUUM_LEVEL_D = 0x0105;  // Actual vacuum D (VGP20)
  static constexpr uint16_t REG_QC_ERROR = 0x0108;
  
  uint8_t num_channels_;
  
  uint8_t extractChannelGripStatus(uint16_t status_reg, uint8_t channel);
  uint8_t extractChannelReleaseStatus(uint16_t status_reg, uint8_t channel);
};

} // namespace onrobot_driver

#endif // ONROBOT_DRIVER__VG_DRIVER_HPP_
