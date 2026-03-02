#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
source /home/georg/projects/ViDeG/.venv/bin/activate
python3 /home/georg/projects/ViDeG/scripts/videg_console_server.py --cloud ws://44.253.251.75:8766
