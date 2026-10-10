"""
Show the rover in RViz.

robot_state_publisher publishes the expanded description (/robot_description, /tf_static, and
/tf for the joints that move), joint_state_publisher_gui gives every joint a slider, and rviz2
opens with rviz/view.rviz:

    ros2 launch rover_description view.launch.py

Arguments: gui (the sliders; false: joint_state_publisher without a window), rviz, and
rover_yaml (the numbers, default config/rover.yaml). Where there is no display (Linux without
DISPLAY or WAYLAND_DISPLAY: a CI runner, a shell over SSH) gui and rviz default to false, and the
launch says so.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def headless(platform=None, environ=None):
    """Return whether there is no display to open a window on (Linux without X11 or Wayland)."""
    platform = sys.platform if platform is None else platform
    environ = os.environ if environ is None else environ
    return platform.startswith('linux') and not (environ.get('DISPLAY') or
                                                 environ.get('WAYLAND_DISPLAY'))


def generate_launch_description():
    share = get_package_share_directory('rover_description')
    windows = 'false' if headless() else 'true'
    gui, rviz = LaunchConfiguration('gui'), LaunchConfiguration('rviz')
    description = ParameterValue(
        Command(['xacro ', os.path.join(share, 'urdf', 'rover.urdf.xacro'),
                 ' rover_yaml:=', LaunchConfiguration('rover_yaml')]),
        value_type=str)
    actions = [
        DeclareLaunchArgument('gui', default_value=windows,
                              description='joint_state_publisher_gui (true) or without a window'),
        DeclareLaunchArgument('rviz', default_value=windows, description='open rviz2'),
        DeclareLaunchArgument('rover_yaml',
                              default_value=os.path.join(share, 'config', 'rover.yaml'),
                              description="the rover's numbers"),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'view.rviz')],
             condition=IfCondition(rviz)),
    ]
    if headless():
        actions.insert(0, LogInfo(msg='no display (DISPLAY and WAYLAND_DISPLAY are unset): '
                                      'gui and rviz default to false'))
    return LaunchDescription(actions)
