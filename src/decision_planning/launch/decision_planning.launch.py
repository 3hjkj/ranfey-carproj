# decision_planning/launch/decision_planning.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, FindPackageShare


def generate_launch_description():
    # $(find route_record)/global_trace.txt  →  ROS 2 等价
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
                # 与 <param name="global_path" …/> 等价
                parameters=[{"global_path": global_trace_path}],
                respawn=True,  # <node … respawn="true" />
                # cwd 在 ROS 2 可直接给绝对/相对路径；若无特殊需求可省略
                # cwd='/tmp'           # ← 如一定要指定工作目录，可手动写路径
            )
        ]
    )
