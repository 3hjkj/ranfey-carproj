from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory("gps_localization")

    return LaunchDescription(
        [
            Node(
                package="gps_localization",
                executable="gps_localization_exec",  # 注意可执行名需和 CMake 中一致
                name="gps_localization",
                output="screen",
                parameters=[
                    {"can_start": os.path.join(package_dir, "can_start.sh")},
                    {"can_down": os.path.join(package_dir, "can_down.sh")},
                ],
                # respawn=True,  # 如果使用 ROS 2 Foxy 或更高版本，可以加上这个字段
            )
        ]
    )
