#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export CYCLONEDDS_URI="file://${SCRIPT_DIR}/cyclonedds_jetson.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_TYPE_HASH_CHECK=1

python3 "${SCRIPT_DIR}/wasd_teleop.py"
