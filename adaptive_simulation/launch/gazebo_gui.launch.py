from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    world = LaunchConfiguration('world')
    urdf  = LaunchConfiguration('urdf')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(
            os.getenv('COLCON_CURRENT_PREFIX', ''), 'share', 'adaptive_simulation', 'worlds', 'empty.world')),
        DeclareLaunchArgument('urdf', default_value=os.path.join(
            os.getenv('COLCON_CURRENT_PREFIX', ''), 'share', 'adaptive_simulation', 'urdf', 'one_link.urdf.xacro')),

        # Gazebo GUI
        ExecuteProcess(cmd=['gazebo', '--verbose', world], output='screen'),

        # 以 gazebo_ros factory spawn 模型
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'one_link', '-file', urdf],
            output='screen'
        ),

        # ros2_control node (Classic)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[os.path.join(
                os.getenv('COLCON_CURRENT_PREFIX', ''), 'share', 'adaptive_bringup', 'config', 'gazebo_controller.yaml')],
            output='screen'
        ),

        # 先帶 joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # 再帶我們的 adaptive_controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['adaptive_controller'],
            output='screen'
        ),
    ])
