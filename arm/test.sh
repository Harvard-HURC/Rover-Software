#!/usr/bin/env bash
set -euo pipefail
arm_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$arm_root"
# Never discover the hardware ROS graph or open a CAN device during tests.
export ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1
export QT_QPA_PLATFORM=offscreen
# Missing ROS/Qt dependencies must fail the task, not turn whole suites into skips.
python -c 'import rclpy, numpy; from PyQt5.QtWidgets import QApplication; from sensor_msgs.msg import JointState'
env -u PYTHONPATH -u PYTHONHOME vendor/i2rt/.venv/bin/python -m unittest -v test_yam_commands
python -m unittest -v test_yam_command_gui test_yam_command_ros test_yam_feedback_ros
