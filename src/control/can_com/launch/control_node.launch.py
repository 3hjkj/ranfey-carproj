from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    control_node_dir = get_package_share_directory('control_node')

    return LaunchDescription([
        Node(
            package='control_node',
            executable='control_node_exec',
            name='control_node',
            output='screen',
            parameters=[
                {'can_start': os.path.join(control_node_dir, 'can_start.sh')},
                {'can_down': os.path.join(control_node_dir, 'can_down.sh')}
            ],
            respawn=True
        )
    ])

