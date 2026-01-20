#!/bin/bash

# OnRobot ROS2 Driver Build Script

set -e

echo "================================================"
echo "OnRobot ROS2 Driver - Build Starting"
echo "================================================"
echo ""

# Verify current directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 1. Install dependencies
echo "[1/4] Installing ROS2 dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# 2. Clean previous build (optional)
if [ "$1" == "clean" ]; then
    echo "[2/4] Cleaning previous build files..."
    rm -rf build install log
else
    echo "[2/4] Skipping build file cleanup (for clean build: ./build.sh clean)"
fi

# 3. colcon build
echo "[3/4] Running colcon build..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. Setup environment
echo "[4/4] Setting up environment variables..."
source install/setup.bash

echo ""
echo "================================================"
echo "Build Complete!"
echo "================================================"
echo ""
echo "Run the following command to setup environment variables:"
echo "  source install/setup.bash"
echo ""
echo "Or to setup automatically:"
echo "  echo 'source $SCRIPT_DIR/install/setup.bash' >> ~/.bashrc"
echo ""
echo "To run the gripper:"
echo "  ros2 launch onrobot_driver onrobot.launch.py"
echo ""
