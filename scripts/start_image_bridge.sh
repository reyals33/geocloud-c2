#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run ros_gz_image image_bridge /camera/image_raw --ros-args --remap /camera/image_raw:=/tb3_2/camera/image_raw
