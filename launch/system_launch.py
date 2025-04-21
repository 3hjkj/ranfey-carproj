from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    perception_launch = os.path.join(
        get_package_share_directory('perception'),
        'launch', 'perception.launch.py'
    )
    rfans_driver_launch = os.path.join(
        get_package_share_directory('rfans_driver'),
        'launch', 'node_manager.launch.py'
    )

    return LaunchDescription([
        Node(
            package='gps_localization',
            executable='gps_localization',
            name='gps_localization',
            output='screen'
        ),
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen'
        ),
        Node(
            package='decision_planning',
            executable='decision_planning',
            name='decision_planning',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rfans_driver_launch)
        )
    ])

