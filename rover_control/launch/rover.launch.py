"""
The real rover's ROS 2 stack (design spec 2026-10-09, section 6.3):

    ros2 launch rover_control rover.launch.py

robot_state_publisher on rover_description's description (urdf/rover.urdf.xacro with its ros2_control block, whose
hardware interface is rover_control's RoverHardwareStub until the motor controllers are chosen: zero motion, and in
the log the commands it would send), ros2_control_node with config/controllers.yaml, and the controllers
(launch/controllers.launch.py): joint_state_broadcaster (/joint_states) and the drive controller, which takes /cmd_vel
(geometry_msgs/TwistStamped) and publishes /odom and odom -> base_link. The simulation runs the same controllers.yaml
(rover_sim's sim.launch.py).

Arguments: controller (the drive controller in controllers.yaml, default drive_controller), hardware (the
ros2_control hardware plugin, default rover_control/RoverHardwareStub), rover_yaml (the rover's numbers, default
rover_description's config/rover.yaml; a relative path is from the folder the launch runs in).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = Path(__file__).resolve().parent.parent  # share/rover_control, or the package's source folder


def generate_launch_description():
    description = get_package_share_directory('rover_description')
    controllers = str(PACKAGE / 'config' / 'controllers.yaml')
    robot_description = ParameterValue(Command([
        'xacro ', os.path.join(description, 'urdf', 'rover.urdf.xacro'),
        ' rover_yaml:=', PathJoinSubstitution([os.getcwd(), LaunchConfiguration('rover_yaml')]),
        ' ros2_control_plugin:=', LaunchConfiguration('hardware')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='drive_controller',
                              description='the drive controller in controllers.yaml'),
        DeclareLaunchArgument('hardware', default_value='rover_control/RoverHardwareStub',
                              description='the ros2_control hardware plugin'),
        DeclareLaunchArgument('rover_yaml',
                              default_value=os.path.join(description, 'config', 'rover.yaml'),
                              description="the rover's numbers (a relative path: from the current "
                                          'folder)'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(PACKAGE / 'launch' / 'controllers.launch.py')),
            launch_arguments={'controller': LaunchConfiguration('controller'),
                              'controllers': controllers}.items()),
    ])
