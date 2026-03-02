#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export LIBGL_ALWAYS_SOFTWARE=1
export TURTLEBOT3_MODEL=waffle
ros2 launch /home/georg/projects/ViDeG/launch/fleet_sim.launch.py headless:=true enable_nav2:=false enable_slam:=false
