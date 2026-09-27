from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sim_sensors_node = Node(
        package="sim_sensors",
        executable="sim_sensors",
        name="sim_sensors",
        output="screen",
        parameters=[{
            "rate_hz": 10.0,
            "lidar_topic": "/points_raw",
            "yolo_topic": "/perception/yolo_boxes",
            "moving_x0": 30.0,
            "moving_vx": -5.0,
            "moving_y": -2.0,
            "static_x": 15.0,
            "static_y": 3.0,
        }],
        emulate_tty=True,
    )
    return LaunchDescription([sim_sensors_node])
