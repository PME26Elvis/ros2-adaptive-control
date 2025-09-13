from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=["config/ros2_control_mock.yaml"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["adaptive_controller"],
            output="screen",
        ),
    ])
