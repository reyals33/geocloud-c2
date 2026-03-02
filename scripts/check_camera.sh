#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
echo "--- ROS2 camera topic ---"
ros2 topic hz /tb3_2/camera/image_raw --window 5
