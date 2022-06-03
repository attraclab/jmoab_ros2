from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()

	atcart8_node = Node(
		package="jmoab_ros2",
		executable="atcart8",
		name="atcart8")

	imu_node = Node(
		package="jmoab_ros2",
		executable="bno055",
		name="imu")

	base_to_imu_node = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
		name="base_to_imu_tf")

	ld.add_action(atcart8_node)
	ld.add_action(imu_node)
	ld.add_action(base_to_imu_node)

	return ld