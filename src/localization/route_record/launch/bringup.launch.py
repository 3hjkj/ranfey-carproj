from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rfans_driver_dir = get_package_share_directory("rfans_driver")
    ndt_localizer_dir = get_package_share_directory("ndt_localizer")
    can_com_dir = get_package_share_directory("can_com")

    return LaunchDescription(
        [
            # Include rfans_driver launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rfans_driver_dir, "launch", "node_manager.launch.py")
                )
            ),
            # Include ndt_localizer launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ndt_localizer_dir, "launch", "ndt_localizer.launch.py")
                )
            ),
            # Node: can_com_pub
            Node(
                package="can_com",
                executable="can_com_pub",
                name="can_com_pub",
                output="screen",
            ),
            # Node: grid_cluster.py
            Node(
                package="can_com",
                executable="grid_cluster.py",
                name="pcl_listener",
                output="screen",
            ),
        ]
    )
