#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

echo "=== odom topic info ==="
ros2 topic info /tb3_2/odom --verbose

echo ""
echo "=== odom hz (5s sample) ==="
timeout 5 ros2 topic hz /tb3_2/odom || true

echo ""
echo "=== odom single message ==="
timeout 3 ros2 topic echo /tb3_2/odom --once || echo "NO ODOM MESSAGE RECEIVED"
