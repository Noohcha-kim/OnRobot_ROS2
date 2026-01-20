#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "onrobot_driver/modbus_tcp_client.hpp"
#include "onrobot_driver/rg_driver.hpp"
#include "onrobot_driver/vg_driver.hpp"
#include "onrobot_driver/mg10_driver.hpp"
#include "onrobot_driver/fg2_driver.hpp"
#include "onrobot_driver/fg3_driver.hpp"
#include "onrobot_driver/fgp20_driver.hpp"
#include "onrobot_driver/hex_driver.hpp"
#include "onrobot_msgs/msg/rg_status.hpp"
#include "onrobot_msgs/msg/rg_command.hpp"
#include "onrobot_msgs/msg/vg_status.hpp"
#include "onrobot_msgs/msg/vg_command.hpp"
#include "onrobot_msgs/msg/mg10_status.hpp"
#include "onrobot_msgs/msg/mg10_command.hpp"
#include "onrobot_msgs/msg/fg_status.hpp"
#include "onrobot_msgs/msg/fg_command.hpp"
#include "onrobot_msgs/msg/fg3_status.hpp"
#include "onrobot_msgs/msg/fg3_command.hpp"
#include "onrobot_msgs/msg/hex_wrench.hpp"

class OnRobotNode : public rclcpp::Node
{
public:
  OnRobotNode() : Node("onrobot_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("robot_ip", "192.168.1.1");
    this->declare_parameter<int>("robot_port", 502);
    this->declare_parameter<int>("device_address", 65);
    this->declare_parameter<int>("update_rate", 50);  // Hz
    this->declare_parameter<std::string>("gripper_type", "auto");
    
    // Get parameters
    robot_ip_ = this->get_parameter("robot_ip").as_string();
    robot_port_ = this->get_parameter("robot_port").as_int();
    device_address_ = this->get_parameter("device_address").as_int();
    update_rate_ = this->get_parameter("update_rate").as_int();
    gripper_type_ = this->get_parameter("gripper_type").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Connecting to %s:%d", robot_ip_.c_str(), robot_port_);
    
    // Initialize Modbus client
    modbus_client_ = std::make_shared<onrobot_driver::ModbusTCPClient>(robot_ip_, robot_port_);
    
    if (!modbus_client_->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to Robot Kit");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Connected successfully");
    
    // Initialize gripper driver based on type
    if (gripper_type_ == "rg2" || gripper_type_ == "rg6") {
      initializeRGDriver();
    }
    else if (gripper_type_ == "vg10" || gripper_type_ == "vgc10" || 
             gripper_type_ == "vgp20" || gripper_type_ == "vgp30") {
      initializeVGDriver();
    }
    else if (gripper_type_ == "mg10") {
      initializeMG10Driver();
    }
    else if (gripper_type_ == "fg2" || gripper_type_ == "2fg7" || gripper_type_ == "2fg14") {
      initializeFG2Driver();
    }
    else if (gripper_type_ == "fgp20" || gripper_type_ == "2fgp20") {
      initializeFGP20Driver();
    }
    else if (gripper_type_ == "fg3" || gripper_type_ == "3fg15" || gripper_type_ == "3fg25") {
      initializeFG3Driver();
    }
    else if (gripper_type_ == "hex") {
      initializeHEXDriver();
    }
    else if (gripper_type_ == "auto") {
      autoDetectGripper();
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown gripper type: %s", gripper_type_.c_str());
      return;
    }
    
    // Create update timer
    auto update_period = std::chrono::milliseconds(1000 / update_rate_);
    update_timer_ = this->create_wall_timer(
      update_period, std::bind(&OnRobotNode::updateCallback, this));
  }
  
  ~OnRobotNode()
  {
    if (modbus_client_) {
      modbus_client_->disconnect();
    }
  }

private:
  void initializeRGDriver()
  {
    rg_driver_ = std::make_shared<onrobot_driver::RGDriver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!rg_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize RG driver");
      return;
    }
    
    rg_status_pub_ = this->create_publisher<onrobot_msgs::msg::RGStatus>("rg_status", 10);
    rg_command_sub_ = this->create_subscription<onrobot_msgs::msg::RGCommand>(
      "rg_command", 10,
      std::bind(&OnRobotNode::rgCommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "RG gripper initialized successfully");
  }
  
  void initializeVGDriver()
  {
    vg_driver_ = std::make_shared<onrobot_driver::VGDriver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!vg_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize VG driver");
      return;
    }
    
    vg_status_pub_ = this->create_publisher<onrobot_msgs::msg::VGStatus>("vg_status", 10);
    vg_command_sub_ = this->create_subscription<onrobot_msgs::msg::VGCommand>(
      "vg_command", 10,
      std::bind(&OnRobotNode::vgCommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "VG gripper initialized successfully (%d channels)", 
                vg_driver_->getNumChannels());
  }
  
  void initializeMG10Driver()
  {
    mg10_driver_ = std::make_shared<onrobot_driver::MG10Driver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!mg10_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize MG10 driver");
      return;
    }
    
    mg10_status_pub_ = this->create_publisher<onrobot_msgs::msg::MG10Status>("mg10_status", 10);
    mg10_command_sub_ = this->create_subscription<onrobot_msgs::msg::MG10Command>(
      "mg10_command", 10,
      std::bind(&OnRobotNode::mg10CommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "MG10 gripper initialized successfully");
  }
  
  void initializeFG2Driver()
  {
    fg2_driver_ = std::make_shared<onrobot_driver::FG2Driver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!fg2_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize 2FG driver");
      return;
    }
    
    fg_status_pub_ = this->create_publisher<onrobot_msgs::msg::FGStatus>("fg_status", 10);
    fg_command_sub_ = this->create_subscription<onrobot_msgs::msg::FGCommand>(
      "fg_command", 10,
      std::bind(&OnRobotNode::fgCommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "2FG gripper initialized successfully");
  }
  
  void initializeFGP20Driver()
  {
    fgp20_driver_ = std::make_shared<onrobot_driver::FGP20Driver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!fgp20_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize 2FGP20 driver");
      return;
    }
    
    fg_status_pub_ = this->create_publisher<onrobot_msgs::msg::FGStatus>("fg_status", 10);
    fg_command_sub_ = this->create_subscription<onrobot_msgs::msg::FGCommand>(
      "fg_command", 10,
      std::bind(&OnRobotNode::fgCommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "2FGP20 gripper (with vacuum) initialized successfully");
  }
  
  void initializeFG3Driver()
  {
    fg3_driver_ = std::make_shared<onrobot_driver::FG3Driver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!fg3_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize 3FG driver");
      return;
    }
    
    fg3_status_pub_ = this->create_publisher<onrobot_msgs::msg::FG3Status>("fg3_status", 10);
    fg3_command_sub_ = this->create_subscription<onrobot_msgs::msg::FG3Command>(
      "fg3_command", 10,
      std::bind(&OnRobotNode::fg3CommandCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "3FG gripper initialized successfully");
  }
  
  void initializeHEXDriver()
  {
    hex_driver_ = std::make_shared<onrobot_driver::HEXDriver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    if (!hex_driver_->initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize HEX driver");
      return;
    }
    
    hex_wrench_pub_ = this->create_publisher<onrobot_msgs::msg::HEXWrench>("hex_wrench", 10);
    
    RCLCPP_INFO(this->get_logger(), "HEX sensor initialized successfully");
  }
  
  void autoDetectGripper()
  {
    // Read product code to auto-detect gripper type
    // Use RG driver temporarily just to read product code
    auto temp_driver = std::make_shared<onrobot_driver::RGDriver>(
      modbus_client_, static_cast<uint8_t>(device_address_));
    
    uint16_t product_code;
    if (!temp_driver->readProductCode(product_code)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read product code for auto-detection");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Detected product code: 0x%04X", product_code);
    
    // Detect gripper type based on product code
    if (product_code == 0x20 || product_code == 0x21) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: RG gripper");
      initializeRGDriver();
    }
    else if (product_code >= 0x10 && product_code <= 0x13) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: VG gripper");
      initializeVGDriver();
    }
    else if (product_code == 0xA0) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: MG10 gripper");
      initializeMG10Driver();
    }
    else if (product_code == 0x30 || product_code == 0x31) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: 2FG7/2FG14 gripper");
      initializeFG2Driver();
    }
    else if (product_code == 0x32) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: 2FGP20 gripper (with vacuum)");
      initializeFGP20Driver();
    }
    else if (product_code >= 0x40 && product_code <= 0x41) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: 3FG gripper");
      initializeFG3Driver();
    }
    else if (product_code == 0x50 || product_code == 0x51) {
      RCLCPP_INFO(this->get_logger(), "Auto-detected: HEX sensor");
      initializeHEXDriver();
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown product code: 0x%04X", product_code);
    }
  }
  
  void updateCallback()
  {
    if (rg_driver_) {
      onrobot_msgs::msg::RGStatus status;
      if (rg_driver_->readStatus(status)) {
        status.header.stamp = this->now();
        status.header.frame_id = "gripper_link";
        rg_status_pub_->publish(status);
      }
    }
    else if (vg_driver_) {
      onrobot_msgs::msg::VGStatus status;
      if (vg_driver_->readStatus(status)) {
        status.header.stamp = this->now();
        status.header.frame_id = "gripper_link";
        vg_status_pub_->publish(status);
      }
    }
    else if (mg10_driver_) {
      onrobot_msgs::msg::MG10Status status;
      if (mg10_driver_->readStatus(status)) {
        status.header.stamp = this->now();
        status.header.frame_id = "gripper_link";
        mg10_status_pub_->publish(status);
      }
    }
    else if (fg2_driver_) {
      onrobot_msgs::msg::FGStatus status;
      if (fg2_driver_->readStatus(status)) {
        status.header.stamp = this->now();
        status.header.frame_id = "gripper_link";
        fg_status_pub_->publish(status);
      }
    }
    else if (fgp20_driver_) {
      onrobot_msgs::msg::FGStatus status;
      if (fgp20_driver_->readStatus(status)) {
        status.header.stamp = this->now();
        status.header.frame_id = "gripper_link";
        fg_status_pub_->publish(status);
      }
    }
    else if (fg3_driver_) {
      onrobot_msgs::msg::FG3Status status;
      if (fg3_driver_->readStatus(status)) {
        status.header.stamp = this->now();
        status.header.frame_id = "gripper_link";
        fg3_status_pub_->publish(status);
      }
    }
    else if (hex_driver_) {
      onrobot_msgs::msg::HEXWrench wrench;
      if (hex_driver_->readWrench(wrench)) {
        hex_wrench_pub_->publish(wrench);
      }
    }
  }
  
  void rgCommandCallback(const onrobot_msgs::msg::RGCommand::SharedPtr msg)
  {
    if (!rg_driver_) return;
    
    switch (msg->command) {
      case onrobot_msgs::msg::RGCommand::GRIP:
        rg_driver_->grip(msg->target_width, msg->target_force);
        break;
      case onrobot_msgs::msg::RGCommand::GRIP_WITH_OFFSET:
        rg_driver_->gripWithOffset(msg->target_width, msg->target_force);
        break;
      case onrobot_msgs::msg::RGCommand::STOP:
        rg_driver_->stop();
        break;
    }
  }
  
  void vgCommandCallback(const onrobot_msgs::msg::VGCommand::SharedPtr msg)
  {
    if (!vg_driver_) return;
    vg_driver_->setControlFloat(msg->control_mode, msg->target_vacuum);
    if (!msg->require_grip.empty()) {
      vg_driver_->setRequireGrip(msg->require_grip);
    }
    if (!msg->ignore_channel.empty()) {
      vg_driver_->setIgnoreChannel(msg->ignore_channel);
    }
  }
  
  void mg10CommandCallback(const onrobot_msgs::msg::MG10Command::SharedPtr msg)
  {
    if (!mg10_driver_) return;
    
    switch (msg->command) {
      case 0: // DISENGAGE
        mg10_driver_->disengage();
        break;
      case 1: // ENGAGE
        mg10_driver_->engage(static_cast<uint8_t>(msg->target_strength));
        break;
      case 2: // ENGAGE_SMART_GRIP
        mg10_driver_->engageSmartGrip(static_cast<uint8_t>(msg->target_strength));
        break;
      case 6: // AUTO_CALIBRATE
        mg10_driver_->autoCalibrate();
        break;
    }
  }
  
  void fgCommandCallback(const onrobot_msgs::msg::FGCommand::SharedPtr msg)
  {
    if (fg2_driver_) {
      switch (msg->command) {
        case 1: // GRIP
          fg2_driver_->grip(msg->target_diameter, msg->target_force);
          break;
        case 3: // STOP
          fg2_driver_->stop();
          break;
      }
    }
    else if (fgp20_driver_) {
      switch (msg->command) {
        case 1: // GRIP
          fgp20_driver_->grip(msg->target_diameter, msg->target_force);
          break;
        case 3: // STOP
          fgp20_driver_->stop();
          break;
      }
    }
  }
  
  void fg3CommandCallback(const onrobot_msgs::msg::FG3Command::SharedPtr msg)
  {
    if (!fg3_driver_) return;
    
    switch (msg->command) {
      case 1: // GRIP
        fg3_driver_->grip(msg->target_diameter, msg->target_force, msg->grip_type);
        break;
      case 2: // MOVE
        fg3_driver_->move(msg->target_diameter, msg->grip_type);
        break;
      case 4: // STOP
        fg3_driver_->stop();
        break;
      case 5: // FLEXIBLE_GRIP
        fg3_driver_->flexibleGrip(msg->target_diameter, msg->target_force, msg->grip_type);
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "Unknown 3FG command: %d", msg->command);
        break;
    }
  }
  
  // Parameters
  std::string robot_ip_;
  int robot_port_;
  int device_address_;
  int update_rate_;
  std::string gripper_type_;
  
  // Modbus client
  std::shared_ptr<onrobot_driver::ModbusTCPClient> modbus_client_;
  
  // Gripper drivers
  std::shared_ptr<onrobot_driver::RGDriver> rg_driver_;
  std::shared_ptr<onrobot_driver::VGDriver> vg_driver_;
  std::shared_ptr<onrobot_driver::MG10Driver> mg10_driver_;
  std::shared_ptr<onrobot_driver::FG2Driver> fg2_driver_;
  std::shared_ptr<onrobot_driver::FG3Driver> fg3_driver_;
  std::shared_ptr<onrobot_driver::FGP20Driver> fgp20_driver_;
  std::shared_ptr<onrobot_driver::HEXDriver> hex_driver_;
  
  // Publishers
  rclcpp::Publisher<onrobot_msgs::msg::RGStatus>::SharedPtr rg_status_pub_;
  rclcpp::Publisher<onrobot_msgs::msg::VGStatus>::SharedPtr vg_status_pub_;
  rclcpp::Publisher<onrobot_msgs::msg::MG10Status>::SharedPtr mg10_status_pub_;
  rclcpp::Publisher<onrobot_msgs::msg::FGStatus>::SharedPtr fg_status_pub_;
  rclcpp::Publisher<onrobot_msgs::msg::FG3Status>::SharedPtr fg3_status_pub_;
  rclcpp::Publisher<onrobot_msgs::msg::HEXWrench>::SharedPtr hex_wrench_pub_;
  
  // Subscribers
  rclcpp::Subscription<onrobot_msgs::msg::RGCommand>::SharedPtr rg_command_sub_;
  rclcpp::Subscription<onrobot_msgs::msg::VGCommand>::SharedPtr vg_command_sub_;
  rclcpp::Subscription<onrobot_msgs::msg::MG10Command>::SharedPtr mg10_command_sub_;
  rclcpp::Subscription<onrobot_msgs::msg::FGCommand>::SharedPtr fg_command_sub_;
  rclcpp::Subscription<onrobot_msgs::msg::FG3Command>::SharedPtr fg3_command_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OnRobotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
