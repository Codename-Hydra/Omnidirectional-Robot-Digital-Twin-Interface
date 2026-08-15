#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export CYCLONEDDS_URI="file://${SCRIPT_DIR}/cyclonedds_jetson.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_TYPE_HASH_CHECK=1

if [ ! -e /dev/ttyUSB0 ]; then
    echo "Starting PL2303 user-space driver (usb_bridge.py)..."
    echo 111111 | sudo -S nohup python3 "${SCRIPT_DIR}/usb_bridge.py" > /dev/null 2>&1 &
    sleep 2
fi

echo "Starting Jetson Unified Robot Bridge..."
python3 "${SCRIPT_DIR}/jetson_robot_bridge.py"
