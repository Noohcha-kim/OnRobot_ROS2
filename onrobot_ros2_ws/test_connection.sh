#!/bin/bash

# OnRobot Gripper Connection Test Script

echo "================================================"
echo "OnRobot Gripper Connection Test"
echo "================================================"
echo ""

# Parameter input
read -p "Robot Kit IP address [192.168.1.1]: " ROBOT_IP
ROBOT_IP=${ROBOT_IP:-192.168.1.1}

read -p "Device Address [65]: " DEVICE_ADDR
DEVICE_ADDR=${DEVICE_ADDR:-65}

read -p "Gripper type (rg2/rg6/vg10/mg10 etc.) [rg2]: " GRIPPER_TYPE
GRIPPER_TYPE=${GRIPPER_TYPE:-rg2}

echo ""
echo "Configuration:"
echo "  IP: $ROBOT_IP"
echo "  Device Address: $DEVICE_ADDR"
echo "  Gripper Type: $GRIPPER_TYPE"
echo ""

# 1. Network connection test
echo "[1/3] Testing network connection..."
if ping -c 1 -W 2 $ROBOT_IP &> /dev/null; then
    echo "✓ Network connection successful"
else
    echo "✗ Network connection failed!"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Verify Robot Kit power is on"
    echo "  2. Check Ethernet cable connection"
    echo "  3. Verify PC network configuration:"
    echo "     - IP: 192.168.1.xxx (same subnet as Robot Kit)"
    echo "     - Netmask: 255.255.255.0"
    exit 1
fi

# 2. Modbus TCP port test
echo "[2/3] Testing Modbus TCP port..."
if command -v nc &> /dev/null; then
    if nc -zv -w 2 $ROBOT_IP 502 &> /dev/null; then
        echo "✓ Modbus TCP port (502) open"
    else
        echo "✗ Modbus TCP port (502) closed"
        echo "  Robot Kit may not be properly booted."
    fi
else
    echo "⚠ Skipping port test (nc command not found)"
    echo "  To install: sudo apt-get install netcat"
fi

# 3. Run ROS2 node
echo "[3/3] Starting ROS2 node..."
echo ""
echo "The gripper will be launched with the following command:"
echo ""
echo "ros2 launch onrobot_driver onrobot.launch.py \\"
echo "    robot_ip:=$ROBOT_IP \\"
echo "    device_address:=$DEVICE_ADDR \\"
echo "    gripper_type:=$GRIPPER_TYPE"
echo ""
read -p "Do you want to run now? (y/n): " RUN_NOW

if [ "$RUN_NOW" == "y" ] || [ "$RUN_NOW" == "Y" ]; then
    source install/setup.bash
    ros2 launch onrobot_driver onrobot.launch.py \
        robot_ip:=$ROBOT_IP \
        device_address:=$DEVICE_ADDR \
        gripper_type:=$GRIPPER_TYPE
else
    echo ""
    echo "To run later, copy and use the command above."
fi
