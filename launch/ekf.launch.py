from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# pkg_path = get_package_share_directory('jmoab_ros2')
# pkg_path_list = pkg_path.split("/")
# config_path = os.path.join(("/"+pkg_path_list[1]),pkg_path_list[2],pkg_path_list[3],\
# 							"src", pkg_path_list[5], "config")


def generate_launch_description():
	return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('jmoab_ros2'),'config','ekf.yaml')],
           ),
])