#!/bin/bash
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=30
export CYCLONEDDS_URI="file://${DIR}/configs/cyclonedds_laptop.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DISABLE_TYPE_HASH_CHECK=1
export ROS_LOCALHOST_ONLY=0

python3 "${DIR}/wasd_teleop.py"
