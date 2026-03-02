#!/bin/bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

echo "=== cv2 check ==="
source /home/georg/projects/ViDeG/.venv/bin/activate
python3 -c "import cv2; print('cv2 OK:', cv2.__version__)" 2>/dev/null || echo "cv2 NOT FOUND"

echo ""
echo "=== camera topic info ==="
ros2 topic info /tb3_2/camera/image_raw --verbose

echo ""
echo "=== camera topic hz (5s sample) ==="
timeout 5 ros2 topic hz /tb3_2/camera/image_raw || true
