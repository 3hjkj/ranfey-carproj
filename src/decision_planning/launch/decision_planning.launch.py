from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    global_trace_path = PathJoinSubstitution(
        [FindPackageShare("route_record"), "global_trace.txt"]
    )

    return LaunchDescription(
        [
            Node(
                package="decision_planning",
                executable="decision_planning",
                name="decision_planning",
                output="screen",
                parameters=[{"global_path": global_trace_path}],
                respawn=True,
            )
        ]
    )
