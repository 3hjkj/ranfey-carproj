from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ck_camera_sdk',
            executable='camera_node',
            name='ck_camera_node',
            output='screen',
            parameters=[  # 如需参数可写这里
            ]
        )
    ])
