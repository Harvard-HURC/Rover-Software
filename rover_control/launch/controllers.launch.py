"""
The rover's controllers, spawned into a running controller manager: rover.launch.py's on the rover, the one
gz_ros2_control runs inside Gazebo in the simulation (rover_sim's sim.launch.py includes this file):

    ros2 launch rover_control controllers.launch.py

joint_state_broadcaster and the drive controller, their parameters from config/controllers.yaml, the drive
controller's ~/cmd_vel and ~/odom on the topic contract's /cmd_vel and /odom (docs/topics.md).

Arguments: controller (the drive controller in controllers.yaml, default drive_controller), controllers (the file,
default this package's config/controllers.yaml), timeout ([s] for the controller manager to come up, default 120: a
big world loads slowly).
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = Path(__file__).resolve().parent.parent  # share/rover_control, or the package's source folder


def spawner_arguments(controller, controllers, timeout):
    """The spawner's arguments for joint_state_broadcaster and the drive controller `controller`."""
    return ['joint_state_broadcaster', controller, '--param-file', controllers,
            '--controller-manager', '/controller_manager', '--controller-manager-timeout', timeout,
            f'--controller-ros-args=-r /{controller}/cmd_vel:=/cmd_vel -r /{controller}/odom:=/odom']


def spawn(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    return [Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=spawner_arguments(arg('controller'), arg('controllers'), arg('timeout')))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='drive_controller',
                              description='the drive controller in controllers.yaml'),
        DeclareLaunchArgument('controllers',
                              default_value=str(PACKAGE / 'config' / 'controllers.yaml'),
                              description="the controllers' parameters"),
        DeclareLaunchArgument('timeout', default_value='120',
                              description='[s] to wait for the controller manager'),
        OpaqueFunction(function=spawn),
    ])
