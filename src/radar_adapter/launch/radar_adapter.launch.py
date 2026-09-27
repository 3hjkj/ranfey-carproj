from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    radar_mode = DeclareLaunchArgument(
        "radar_mode", default_value="sim",
        description="sim（默认，模拟目标）或 autocontrol（订阅 AutocontrolRadardata）"
    )
    return LaunchDescription([
        radar_mode,
        Node(
            package="radar_adapter",
            executable="radar_adapter",
            name="radar_adapter",
            output="screen",
            parameters=[{"radar_mode": LaunchConfiguration("radar_mode")}],
        ),
    ])
