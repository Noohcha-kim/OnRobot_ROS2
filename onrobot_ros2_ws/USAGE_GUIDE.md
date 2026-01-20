# OnRobot ROS2 Driver - Complete Usage Guide

## Table of Contents
1. [Package Overview](#package-overview)
2. [Build and Installation](#build-and-installation)
3. [Usage by Gripper Type](#usage-by-gripper-type)
4. [Programming Guide](#programming-guide)
5. [Advanced Features](#advanced-features)

## Package Overview

### Structure

```
onrobot_ros2_ws/
├── src/
│   ├── onrobot_driver/          # Main driver package
│   │   ├── include/             # C++ header files
│   │   │   └── onrobot_driver/
│   │   │       ├── modbus_tcp_client.hpp    # Modbus TCP client
│   │   │       ├── onrobot_base.hpp         # Base driver class
│   │   │       ├── rg_driver.hpp            # RG2/RG6 driver
│   │   │       ├── vg_driver.hpp            # VG series driver
│   │   │       ├── mg10_driver.hpp          # MG10 driver
│   │   │       ├── fg2_driver.hpp           # 2FG driver
│   │   │       ├── fg3_driver.hpp           # 3FG driver
│   │   │       └── hex_driver.hpp           # HEX F/T sensor
│   │   ├── src/                 # C++ implementation files
│   │   ├── launch/              # Launch files
│   │   ├── config/              # Configuration files
│   │   ├── examples/            # Usage examples
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── onrobot_msgs/            # Message/service definitions
│       ├── msg/                 # Message files
│       ├── srv/                 # Service files
│       ├── CMakeLists.txt
│       └── package.xml
├── build.sh                     # Build script
├── test_connection.sh           # Connection test script
├── README.md                    # Full documentation
└── QUICKSTART.md               # Quick start guide
```

## Build and Installation

### Colcon Build Method

#### 1. Install Dependencies

```bash
cd ~/onrobot_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

#### 2. Build

```bash
# Standard build
colcon build --symlink-install

# Release build (optimized)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clean build
rm -rf build install log
colcon build --symlink-install
```

#### 3. Setup Environment

```bash
# Apply to current terminal only
source install/setup.bash

# Auto-apply to all terminals
echo "source ~/onrobot_ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Using Build Script

```bash
cd ~/onrobot_ros2_ws

# Standard build
./build.sh

# Clean build
./build.sh clean
```

## Usage by Gripper Type

### 1. RG2/RG6 - 2-Finger Electric Gripper

#### Launch

```bash
# RG2
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=rg2

# RG6
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=rg6
```

#### Topics

- **Status**: `/rg_status` (50Hz)
  ```bash
  ros2 topic echo /rg_status
  ```

- **Command**: `/rg_command`
  ```bash
  # Grip (50mm, 100N)
  ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
    "{command: 1, target_width: 50.0, target_force: 100.0}"
  ```

#### Python Example

```python
import rclpy
from rclpy.node import Node
from onrobot_msgs.msg import RGCommand, RGStatus

class RGController(Node):
    def __init__(self):
        super().__init__('rg_controller')
        self.cmd_pub = self.create_publisher(RGCommand, 'rg_command', 10)
        self.status_sub = self.create_subscription(
            RGStatus, 'rg_status', self.status_cb, 10)
    
    def status_cb(self, msg):
        self.get_logger().info(
            f'Width: {msg.actual_width:.1f}mm, '
            f'Grip: {msg.grip_detected}')
    
    def grip(self, width_mm, force_n):
        cmd = RGCommand()
        cmd.command = RGCommand.GRIP
        cmd.target_width = width_mm
        cmd.target_force = force_n
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Grip: {width_mm}mm, {force_n}N')

def main():
    rclpy.init()
    node = RGController()
    
    # Usage example
    node.grip(50.0, 100.0)
    
    rclpy.spin(node)
    rclpy.shutdown()
```

#### Specifications

| Model | Stroke   | Max Force | Payload |
|-------|----------|-----------|---------|
| RG2   | 2-110mm  | 400N      | 2kg     |
| RG6   | 6-160mm  | 1200N     | 6kg     |

### 2. VG10/VGC10/VGP20/VGP30 - Vacuum Grippers

#### Launch

```bash
# VG10 (2 channels)
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=vg10

# VGP20 (4 channels)
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=vgp20

# VGP30 (2 channels, high performance)
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=vgp30
```

#### Python Example

```python
from onrobot_msgs.msg import VGCommand, VGStatus

class VGController(Node):
    def __init__(self):
        super().__init__('vg_controller')
        self.cmd_pub = self.create_publisher(VGCommand, 'vg_command', 10)
    
    def grip_channel(self, channel_num, vacuum_percent):
        """Vacuum grip with specific channel"""
        cmd = VGCommand()
        
        # For 2-channel gripper
        cmd.control_mode = [
            VGCommand.MODE_RELEASE,  # Channel A
            VGCommand.MODE_RELEASE   # Channel B
        ]
        cmd.target_vacuum = [0.0, 0.0]
        cmd.require_grip = [False, False]
        cmd.ignore_channel = [False, False]
        
        # Activate specified channel only
        cmd.control_mode[channel_num] = VGCommand.MODE_GRIP
        cmd.target_vacuum[channel_num] = vacuum_percent
        cmd.require_grip[channel_num] = True
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Channel {channel_num}: {vacuum_percent}% vacuum')
    
    def release_all(self):
        """Release all channels"""
        cmd = VGCommand()
        cmd.control_mode = [VGCommand.MODE_RELEASE] * 2
        cmd.target_vacuum = [0.0] * 2
        cmd.require_grip = [False] * 2
        cmd.ignore_channel = [False] * 2
        self.cmd_pub.publish(cmd)
```

#### Usage Example

```python
# Grip with channel A at 60% vacuum
node.grip_channel(0, 60.0)

# Wait 3 seconds
time.sleep(3.0)

# Release all channels
node.release_all()
```

### 3. MG10 - Magnetic Gripper

#### Launch

```bash
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=mg10
```

#### Python Example

```python
from onrobot_msgs.msg import MG10Command, MG10Status

class MG10Controller(Node):
    def __init__(self):
        super().__init__('mg10_controller')
        self.cmd_pub = self.create_publisher(MG10Command, 'mg10_command', 10)
        self.status_sub = self.create_subscription(
            MG10Status, 'mg10_status', self.status_cb, 10)
    
    def status_cb(self, msg):
        if msg.part_gripped:
            self.get_logger().info('Object gripped!')
        if msg.near_part:
            self.get_logger().info('Object proximity detected')
    
    def engage(self, strength_percent=100.0):
        """Activate magnet"""
        cmd = MG10Command()
        cmd.command = MG10Command.ENGAGE
        cmd.target_strength = strength_percent
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Engage: {strength_percent}%')
    
    def engage_smart_grip(self, strength_percent=100.0):
        """Activate magnet with smart grip"""
        cmd = MG10Command()
        cmd.command = MG10Command.ENGAGE_SMART_GRIP
        cmd.target_strength = strength_percent
        self.cmd_pub.publish(cmd)
        self.get_logger().info(f'Smart Grip: {strength_percent}%')
    
    def disengage(self):
        """Deactivate magnet"""
        cmd = MG10Command()
        cmd.command = MG10Command.DISENGAGE
        cmd.target_strength = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Disengage')
    
    def calibrate(self):
        """Auto calibration"""
        cmd = MG10Command()
        cmd.command = MG10Command.AUTO_CALIBRATE
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Calibrating...')
```

#### Command Line Usage

```bash
# Activate magnet (100% strength)
ros2 topic pub --once /mg10_command onrobot_msgs/msg/MG10Command \
  "{command: 1, target_strength: 100.0}"

# Smart grip
ros2 topic pub --once /mg10_command onrobot_msgs/msg/MG10Command \
  "{command: 2, target_strength: 100.0}"

# Deactivate
ros2 topic pub --once /mg10_command onrobot_msgs/msg/MG10Command \
  "{command: 0, target_strength: 0.0}"
```

### 4. HEX-E/H QC - Force/Torque Sensor

#### Launch

```bash
ros2 launch onrobot_driver onrobot.launch.py \
    gripper_type:=hex \
    device_address:=64
```

#### Python Example

```python
from onrobot_msgs.msg import HEXWrench
from geometry_msgs.msg import Vector3

class HEXMonitor(Node):
    def __init__(self):
        super().__init__('hex_monitor')
        self.wrench_sub = self.create_subscription(
            HEXWrench, 'hex_wrench', self.wrench_cb, 10)
        
        # Zero service (implementation needed)
        self.zero_cli = self.create_client(Empty, 'hex_zero')
    
    def wrench_cb(self, msg):
        fx, fy, fz = msg.force.x, msg.force.y, msg.force.z
        tx, ty, tz = msg.torque.x, msg.torque.y, msg.torque.z
        
        self.get_logger().info(
            f'Force: ({fx:.2f}, {fy:.2f}, {fz:.2f}) N, '
            f'Torque: ({tx:.3f}, {ty:.3f}, {tz:.3f}) Nm')
    
    async def zero_sensor(self):
        """Set sensor zero"""
        req = Empty.Request()
        future = self.zero_cli.call_async(req)
        await future
        self.get_logger().info('Sensor zeroed')
```

## Programming Guide

### C++ Usage

#### 1. Add Package Dependencies

`package.xml`:
```xml
<depend>onrobot_msgs</depend>
<depend>onrobot_driver</depend>
```

`CMakeLists.txt`:
```cmake
find_package(onrobot_msgs REQUIRED)
find_package(onrobot_driver REQUIRED)

ament_target_dependencies(your_node
  rclcpp
  onrobot_msgs
  onrobot_driver
)
```

#### 2. Write C++ Node

```cpp
#include <rclcpp/rclcpp.hpp>
#include "onrobot_msgs/msg/rg_command.hpp"
#include "onrobot_msgs/msg/rg_status.hpp"

class MyGripperController : public rclcpp::Node
{
public:
  MyGripperController() : Node("my_gripper_controller")
  {
    cmd_pub_ = this->create_publisher<onrobot_msgs::msg::RGCommand>(
      "rg_command", 10);
    
    status_sub_ = this->create_subscription<onrobot_msgs::msg::RGStatus>(
      "rg_status", 10,
      std::bind(&MyGripperController::statusCallback, this, 
                std::placeholders::_1));
  }
  
  void grip(float width_mm, float force_n)
  {
    auto cmd = onrobot_msgs::msg::RGCommand();
    cmd.command = onrobot_msgs::msg::RGCommand::GRIP;
    cmd.target_width = width_mm;
    cmd.target_force = force_n;
    cmd_pub_->publish(cmd);
    
    RCLCPP_INFO(this->get_logger(), 
                "Grip command: %.1f mm, %.1f N", width_mm, force_n);
  }
  
private:
  void statusCallback(const onrobot_msgs::msg::RGStatus::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), 
                        *this->get_clock(), 1000,  // Every 1 second
                        "Width: %.1f mm, Grip: %d",
                        msg->actual_width, msg->grip_detected);
  }
  
  rclcpp::Publisher<onrobot_msgs::msg::RGCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<onrobot_msgs::msg::RGStatus>::SharedPtr status_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyGripperController>();
  
  // Usage example
  node->grip(50.0, 100.0);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### Python Usage

#### 1. Setup.py Dependencies

```python
install_requires=[
    'setuptools',
    'onrobot-msgs',
],
```

#### 2. Write Python Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from onrobot_msgs.msg import RGCommand, RGStatus
import time

class MyGripperController(Node):
    def __init__(self):
        super().__init__('my_gripper_controller')
        
        self.cmd_pub = self.create_publisher(
            RGCommand, 'rg_command', 10)
        
        self.status_sub = self.create_subscription(
            RGStatus, 'rg_status', 
            self.status_callback, 10)
        
        self.current_status = None
    
    def status_callback(self, msg):
        self.current_status = msg
        # Log once per second
        if not hasattr(self, '_last_log') or \
           (time.time() - self._last_log) > 1.0:
            self.get_logger().info(
                f'Width: {msg.actual_width:.1f}mm, '
                f'Grip: {msg.grip_detected}')
            self._last_log = time.time()
    
    def grip(self, width_mm, force_n):
        cmd = RGCommand()
        cmd.command = RGCommand.GRIP
        cmd.target_width = width_mm
        cmd.target_force = force_n
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'Grip: {width_mm}mm, {force_n}N')
    
    def wait_not_busy(self, timeout=10.0):
        """Wait until gripper is in idle state"""
        start = time.time()
        while self.current_status and \
              self.current_status.busy:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                return False
        return True

def main():
    rclpy.init()
    node = MyGripperController()
    
    # Wait for status reception
    while node.current_status is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Grip
    node.grip(50.0, 100.0)
    node.wait_not_busy()
    
    # Continue running
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Features

### 1. Using Dual Quick Changer

When using two grippers simultaneously:

#### Terminal 1: Primary Gripper

```bash
ros2 launch onrobot_driver onrobot.launch.py \
    gripper_type:=rg2 \
    device_address:=66 \
    namespace:=gripper_primary
```

#### Terminal 2: Secondary Gripper

```bash
ros2 launch onrobot_driver onrobot.launch.py \
    gripper_type:=vg10 \
    device_address:=67 \
    namespace:=gripper_secondary
```

#### Control Both Grippers with Python

```python
class DualGripperController(Node):
    def __init__(self):
        super().__init__('dual_gripper_controller')
        
        # Primary gripper (RG2)
        self.rg_cmd_pub = self.create_publisher(
            RGCommand, 'gripper_primary/rg_command', 10)
        
        # Secondary gripper (VG10)
        self.vg_cmd_pub = self.create_publisher(
            VGCommand, 'gripper_secondary/vg_command', 10)
    
    def grip_with_both(self):
        # Side grip with RG2
        rg_cmd = RGCommand()
        rg_cmd.command = RGCommand.GRIP
        rg_cmd.target_width = 50.0
        rg_cmd.target_force = 100.0
        self.rg_cmd_pub.publish(rg_cmd)
        
        # Top suction with VG10
        vg_cmd = VGCommand()
        vg_cmd.control_mode = [VGCommand.MODE_GRIP] * 2
        vg_cmd.target_vacuum = [60.0] * 2
        vg_cmd.require_grip = [True] * 2
        vg_cmd.ignore_channel = [False] * 2
        self.vg_cmd_pub.publish(vg_cmd)
```

### 2. Dynamic Parameter Changes

```bash
# Change update rate
ros2 param set /onrobot_node update_rate 100

# Change IP address (requires restart)
ros2 param set /onrobot_node robot_ip 192.168.1.50
```

### 3. Customize Launch File

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare parameters
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.1'),
        DeclareLaunchArgument('gripper_type', default_value='rg2'),
        
        # Start node
        Node(
            package='onrobot_driver',
            executable='onrobot_node',
            name='onrobot_node',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'gripper_type': LaunchConfiguration('gripper_type'),
                'update_rate': 50,
            }],
            # Set log level
            arguments=['--ros-args', '--log-level', 'info'],
            # Restart configuration
            respawn=True,
            respawn_delay=2.0,
        )
    ])
```

### 4. Debugging

```bash
# Change log level
ros2 run onrobot_driver onrobot_node --ros-args --log-level debug

# Check topic Hz
ros2 topic hz /rg_status

# Check topic bandwidth
ros2 topic bw /rg_status

# Node information
ros2 node info /onrobot_node

# List parameters
ros2 param list /onrobot_node
```

Now you can fully control all OnRobot grippers with ROS2!
