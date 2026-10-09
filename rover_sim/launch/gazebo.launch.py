import os
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	gz_pkg = get_package_share_directory('ros_gz_sim')

	world_arg = DeclareLaunchArgument(
		'world',
		default_value='empty.sdf',
		description='Name or path of the Gazebo SDF world file'
	)

	gz_args = LaunchConfiguration('world')

	launch_actions = [world_arg]

	if sys.platform == 'darwin':
		print("[INFO] macOS detected: Launching Gazebo as separate Server and GUI processes")

		gz_server = IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(gz_pkg, 'launch', 'gz_server.launch.py')
			),
			launch_arguments={'world_sdf_file': gz_args}.items()
		)

		gz_gui = ExecuteProcess(
			cmd=['gz', 'sim', '-g'],
			output='screen'
		)

		launch_actions.extend([gz_server, gz_gui])
	else:
		gz = IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')
			),
			launch_arguments={'gz_args': gz_args}.items()
		)

		launch_actions.append(gz)

	return LaunchDescription(launch_actions)
