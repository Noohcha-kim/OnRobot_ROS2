# OnRobot ROS2 Driver - Quick Start Guide

## ⚠️ Before You Begin

**Check Firmware Version**: This driver is based on Robot Kit firmware v6.5.2
- Recommended version: v6.5.2
- Firmware update required if version is too low
- Version automatically displayed in logs when driver runs

## Get Started in 5 Minutes

### Step 1: Hardware Connection (2 minutes)

1. **Connect Robot Kit (Compute Box) Power**
   - Connect 24V power adapter to Robot Kit
   - Verify LED turns on

2. **Connect Gripper**
   - Mount gripper on Quick Changer
   - Connection complete when you hear a "click"

3. **Connect Ethernet Cable**
   - Connect Robot Kit's LAN port to PC with Ethernet cable

4. **Configure PC Network**
   ```bash
   # Check current network interface
   ip addr
   
   # Set static IP (example: eth0 interface)
   sudo ip addr add 192.168.1.100/24 dev eth0
   ```

5. **Test Connection**
   ```bash
   ping 192.168.1.1
   ```
   Success if you get a response!

### Step 2: Software Installation (2 minutes)

```bash
# Navigate to workspace
cd ~/onrobot_ros2_ws

# Run build script
./build.sh

# Setup environment variables
source install/setup.bash
```

### Step 3: Run Gripper (1 minute)

#### Option A: Use Automated Test Script

```bash
cd ~/onrobot_ros2_ws
./test_connection.sh
```

Follow the interactive prompts!

#### Option B: Manual Execution

```bash
# Run RG2 gripper
ros2 launch onrobot_driver onrobot.launch.py

# Check status in another terminal
ros2 topic echo /rg_status
```

## Send Your First Command

### Method 1: From Command Line

```bash
# Open gripper (110mm)
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 1, target_width: 110.0, target_force: 40.0}"

# Close gripper (50mm, 100N)
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 1, target_width: 50.0, target_force: 100.0}"
```

### Method 2: Run Python Example

```bash
python3 src/onrobot_driver/examples/rg_gripper_example.py
```

## Key Commands Summary

### Build
```bash
cd ~/onrobot_ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Run
```bash
# Default (RG2)
ros2 launch onrobot_driver onrobot.launch.py

# RG6
ros2 launch onrobot_driver onrobot.launch.py gripper_type:=rg6

# Custom IP
ros2 launch onrobot_driver onrobot.launch.py robot_ip:=192.168.1.1
```

### Status Monitoring
```bash
# Subscribe to status topic
ros2 topic echo /rg_status

# Check frequency
ros2 topic hz /rg_status

# List topics
ros2 topic list
```

### Control
```bash
# Grip (50mm, 100N)
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 1, target_width: 50.0, target_force: 100.0}"

# Stop
ros2 topic pub --once /rg_command onrobot_msgs/msg/RGCommand \
  "{command: 3, target_width: 0.0, target_force: 0.0}"
```

## Troubleshooting

### "Failed to connect" Error

1. **Check Network**
   ```bash
   ping 192.168.1.1
   ```
   
2. **Disable Firewall**
   ```bash
   sudo ufw disable
   ```

3. **Reconnect Cable**
   - Unplug and replug Ethernet cable

### "Failed to initialize" Error

1. **Check Device Address**
   - Quick Changer: 65
   - Dual Quick Changer Primary: 66
   - Dual Quick Changer Secondary: 67

2. **Check Gripper Type**
   - Verify Product Code in logs
   - RG2: 0x20, RG6: 0x21

### Build Errors

```bash
# Clean build
cd ~/onrobot_ros2_ws
rm -rf build install log
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Next Steps

1. **Learn from Example Code**
   - See `src/onrobot_driver/examples/rg_gripper_example.py`

2. **Write Your Own Node**
   - Implement custom control logic in Python or C++

3. **Try Other Grippers**
   - Test other products like VG10, MG10

4. **Explore Advanced Features**
   - Force/Torque sensor (RG2-FT, HEX)
   - Vision system (Eyes)

## Support

- **README**: See full documentation in `~/onrobot_ros2_ws/README.md`
- **Examples**: Check `src/onrobot_driver/examples/` directory
- **Configuration**: See parameters in `src/onrobot_driver/config/onrobot.yaml`

## Key File Locations

```
~/onrobot_ros2_ws/
├── build.sh                    # Build script
├── test_connection.sh          # Connection test script
├── README.md                   # Full documentation
└── src/
    ├── onrobot_driver/         # Main driver package
    │   ├── launch/             # Launch files
    │   ├── config/             # Configuration files
    │   ├── examples/           # Example code
    │   └── include/            # C++ headers
    └── onrobot_msgs/           # Message definitions
```

Congratulations! You're now ready to control OnRobot grippers with ROS2! 🎉
