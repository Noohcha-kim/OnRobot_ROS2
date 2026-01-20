# OnRobot ROS2 Driver

A comprehensive ROS2 driver package supporting all OnRobot gripper products. Controls grippers via Modbus TCP communication through the Robot Kit (Compute Box).

## ⚠️ Important: Firmware Compatibility

This driver is developed based on **OnRobot Connectivity Guide v1.22.0**.

### Robot Kit Firmware Requirements:
- **Recommended Version**: v1.22.0 or higher
- **Minimum Version**: v1.20.0 or higher (some features may be limited)

### How to Check Firmware Version:
```bash
# Automatically displayed in logs when driver runs
ros2 launch onrobot_driver onrobot.launch.py

# Log example:
# [INFO] [onrobot_node]: Connected to Robot Kit at 192.168.1.1:502
# [INFO] [onrobot_node]: Firmware Version: 1.22.0
# [INFO] [onrobot_node]: Product Code: 0x20 (RG2)
```

### If Firmware Version Differs:
1. **Lower version (< v1.20.0)**: Robot Kit firmware update required
2. **Higher version (> v1.22.0)**: Usually compatible, but check latest Connectivity Guide
3. **On compatibility issues**: Contact OnRobot technical support or match firmware version

> **Note**: Register addresses or operations may change with different firmware versions.

## Supported Products

### Grippers
- **RG2/RG6** - 2-finger electric gripper
- **2FG7/2FG14/2FGP20** - 2-finger flexible gripper
- **3FG15/3FG25** - 3-finger gripper
- **VG10/VGC10** - Vacuum gripper (2 channels)
- **VGP20** - Vacuum gripper (4 channels)
- **MG10** - Magnetic gripper



## System Requirements

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS2 Humble
- C++17 
- Python 3.12 
- Robot kit ver 6.5 or later

## Installation

### 1. Create Workspace

```bash
mkdir -p ~/onrobot_ros2_ws/src
cd ~/onrobot_ros2_ws/src
```

### 2. Copy Package

Copy this package to the src directory of your workspace:

```bash
# If the package is already in /home/claude/onrobot_ros2_ws/src
cd ~/onrobot_ros2_ws
```

### 3. Install Dependencies

```bash
cd ~/onrobot_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build

```bash
cd ~/onrobot_ros2_ws
colcon build --symlink-install
```

If the build is successful, you will see a message like:
```
Summary: 2 packages finished [X.Xs]
```

### 5. Setup Environment

```bash
source ~/onrobot_ros2_ws/install/setup.bash
```

**Important**: You must run this command every time you open a new terminal. To make it automatic:

```bash
echo "source ~/onrobot_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Network Configuration

### Robot Kit (Compute Box) IP Configuration

The Robot Kit uses `192.168.1.1` as the default IP. This can be configured with DIP switches:

- **DIP 3 ON + DIP 4 OFF**: Fixed IP (192.168.1.1) - Default setting
- **DIP 3 OFF + DIP 4 ON**: Advanced mode (static/dynamic IP configurable)

### PC Network Configuration

To communicate with the Robot Kit, configure your PC's IP to be on the same subnet:

```bash
# Check network interface
ip addr

# Set static IP (example: 192.168.1.100)
sudo ip addr add 192.168.1.100/24 dev eth0  # Replace eth0 with your actual interface name
```

Or configure via GUI:
1. Settings → Network
2. Select Ethernet connection → ⚙️ (Settings)
3. IPv4 tab
4. Method: Manual
5. Address: `192.168.1.100`
6. Netmask: `255.255.255.0`
7. Gateway: `192.168.1.1`

### Connection Test

```bash
ping 192.168.1.1
```

If you receive a response, the connection is successful.

## Usage

### 1. Basic Execution

```bash
# RG2 gripper (default configuration)
ros2 launch onrobot_driver onrobot.launch.py

# RG6 gripper
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=rg6

# VG10 vacuum gripper
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=vg10

# MG10 magnetic gripper
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=mg10
```

### 2. Custom IP/Port Configuration

```bash
ros2 launch onrobot_driver onrobot.launch.py \
    robot_ip:=192.168.1.1 \
    robot_port:=502 \
    device_address:=65
```

### 3. Device Address Configuration

- `65` (0x41): Quick Changer or HEX-E/H QC
- `66` (0x42): Dual Quick Changer - Primary side
- `67` (0x43): Dual Quick Changer - Secondary side
- `165` (0xA5): Lift100

```bash
# Second gripper on Dual Quick Changer
ros2 launch onrobot_driver onrobot.launch.py device_address:=67
```

## ROS2 Topics and Services

### RG2/RG6 Gripper

#### Topics

**Published Topics:**
- `/rg_status` (`onrobot_msgs/msg/RGStatus`) - Gripper status (50Hz)
  - `busy` - Operation in progress
  - `grip_detected` - Object detection status
  - `actual_width` - Current width (mm)
  - `actual_force` - Current force (N)
  - `safety_error` - Safety error status

**Subscribed Topics:**
- `/rg_command` (`onrobot_msgs/msg/RGCommand`) - Gripper command
  - `GRIP` (1) - Grip
  - `GRIP_WITH_OFFSET` (2) - Grip with offset
  - `STOP` (3) - Stop

### VG Vacuum Gripper

- `/vg_status` (`onrobot_msgs/msg/VGStatus`) - Vacuum status
- `/vg_command` (`onrobot_msgs/msg/VGCommand`) - Vacuum control command

### MG10 Magnetic Gripper

- `/mg10_status` (`onrobot_msgs/msg/MG10Status`) - Magnet status
- `/mg10_command` (`onrobot_msgs/msg/MG10Command`) - Magnet control command

### HEX Force/Torque Sensor

- `/hex_wrench` (`onrobot_msgs/msg/HEXWrench`) - Force/Torque data

## Programming Examples

### Python Example

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from onrobot_msgs.msg import RGCommand, RGStatus

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        
        # Command publisher
        self.cmd_pub = self.create_publisher(RGCommand, 'rg_command', 10)
        
        # Status subscription
        self.status_sub = self.create_subscription(
            RGStatus, 'rg_status', self.status_callback, 10)
        
        self.current_status = None
    
    def status_callback(self, msg):
        self.current_status = msg
        self.get_logger().info(f'Width: {msg.actual_width:.1f}mm, '
                              f'Grip: {msg.grip_detected}')
    
    def grip(self, width_mm, force_n):
        """Grip with gripper"""
        cmd = RGCommand()
        cmd.command = RGCommand.GRIP
        cmd.target_width = width_mm
        cmd.target_force = force_n
        self.cmd_pub.publish(cmd)
    
    def open(self):
        """Open gripper"""
        cmd = RGCommand()
        cmd.command = RGCommand.GRIP
        cmd.target_width = 110.0  # RG2 maximum width
        cmd.target_force = 40.0
        self.cmd_pub.publish(cmd)
    
    def stop(self):
        """Stop gripper"""
        cmd = RGCommand()
        cmd.command = RGCommand.STOP
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = GripperController()
    
    # Grip after 2 seconds
    node.create_timer(2.0, lambda: node.grip(50.0, 100.0))
    
    # Open after 5 seconds
    node.create_timer(5.0, lambda: node.open())
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Example

```cpp
#include <rclcpp/rclcpp.hpp>
#include "onrobot_msgs/msg/rg_command.hpp"
#include "onrobot_msgs/msg/rg_status.hpp"

class GripperController : public rclcpp::Node
{
public:
  GripperController() : Node("gripper_controller")
  {
    // Command publisher
    cmd_pub_ = this->create_publisher<onrobot_msgs::msg::RGCommand>(
      "rg_command", 10);
    
    // Status subscription
    status_sub_ = this->create_subscription<onrobot_msgs::msg::RGStatus>(
      "rg_status", 10,
      std::bind(&GripperController::statusCallback, this, std::placeholders::_1));
  }
  
  void grip(float width_mm, float force_n)
  {
    auto cmd = onrobot_msgs::msg::RGCommand();
    cmd.command = onrobot_msgs::msg::RGCommand::GRIP;
    cmd.target_width = width_mm;
    cmd.target_force = force_n;
    cmd_pub_->publish(cmd);
  }
  
  void open()
  {
    auto cmd = onrobot_msgs::msg::RGCommand();
    cmd.command = onrobot_msgs::msg::RGCommand::GRIP;
    cmd.target_width = 110.0;  // RG2 maximum width
    cmd.target_force = 40.0;
    cmd_pub_->publish(cmd);
  }

private:
  void statusCallback(const onrobot_msgs::msg::RGStatus::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Width: %.1f mm, Grip: %d",
                msg->actual_width, msg->grip_detected);
  }
  
  rclcpp::Publisher<onrobot_msgs::msg::RGCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<onrobot_msgs::msg::RGStatus>::SharedPtr status_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperController>();
  
  // Grip after 2 seconds
  rclcpp::sleep_for(std::chrono::seconds(2));
  node->grip(50.0, 100.0);
  
  // Open after 3 seconds
  rclcpp::sleep_for(std::chrono::seconds(3));
  node->open();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Direct Control from Command Line

```bash
# Check gripper status
ros2 topic echo /rg_status

# Grip command (50mm, 100N)
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 1, target_width: 50.0, target_force: 100.0}"

# Open command (110mm, 40N)
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 1, target_width: 110.0, target_force: 40.0}"

# Stop command
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 3, target_width: 0.0, target_force: 0.0}"
```

## Running Example Scripts

To run the included examples:

```bash
# Grant execute permission to Python example
chmod +x ~/onrobot_ros2_ws/src/onrobot_driver/examples/rg_gripper_example.py

# Run example (onrobot_node must be running first)
python3 ~/onrobot_ros2_ws/src/onrobot_driver/examples/rg_gripper_example.py
```

## Troubleshooting

### Cannot Connect

1. **Check Network**
   ```bash
   ping 192.168.1.1
   ```

2. **Check Firewall**
   ```bash
   sudo ufw allow from 192.168.1.0/24
   ```

3. **Check Modbus TCP Port**
   ```bash
   nc -zv 192.168.1.1 502
   ```

### Gripper Not Responding

1. **Check Device Address** - Varies by Quick Changer type
2. **Check Product Code** - Verify product code in logs
3. **Check Cable Connection** - M8-8pin or M12-12pin connector

### Build Errors

```bash
# Reinstall dependencies
cd ~/onrobot_ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build install log
colcon build
```

## References

- [OnRobot Official Website](https://onrobot.com)
- [Modbus TCP Protocol](https://www.modbus.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

## License

Apache License 2.0

## Contributing

Please submit bug reports and feature requests to GitHub Issues.

## Contact

- Developer: minseung2201@gmail.com
- Company: OnRobot A/S
