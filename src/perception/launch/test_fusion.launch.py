# 端到端融合测试：sim 数据源（无实车）
#   radar_adapter(sim)  → /sensorRawData     毫米波
#   sim_sensors         → /points_raw + /perception/yolo_boxes   激光 + 相机
#   perception          → /perception/lidar_cells（融合栅格） + /perception_objs（融合目标）
#
# 用法：ros2 launch perception test_fusion.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory("perception")
    cfg_dir = os.path.join(share_dir, "config_json")
    log_dir = os.path.join(share_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)

    fusion_enable = LaunchConfiguration("fusion_enable", default="true")

    radar_adapter = Node(
        package="radar_adapter",
        executable="radar_adapter",
        name="radar_adapter",
        output="screen",
        parameters=[{
            "radar_mode": "sim",
            "radar_topic": "/sensorRawData",
            "sim_moving_x0": 30.0,
            "sim_moving_vx": -5.0,
            "sim_moving_y": -2.0,
            "sim_static_x": 15.0,
            "sim_static_y": 3.0,
        }],
        emulate_tty=True,
    )

    sim_sensors = Node(
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

    perception = Node(
        package="perception",
        executable="perception",
        name="perception",
        output="screen",
        parameters=[{
            "cell_config": os.path.join(cfg_dir, "cell.json"),
            "obj_config": os.path.join(cfg_dir, "objs.json"),
            "preprocess_config": os.path.join(cfg_dir, "preprocess.json"),
            "method": 1,
            "topic_name": "fusion",
            "log_path": os.path.join(log_dir, "perception.log"),
            "lidar_topic_name0": "/points_raw",
            "lidar_topic_name1": "/ns2/points_raw",
            "lidar_topic_name2": "/ns3/points_raw",
            "lidar_topic_name3": "/ns4/points_raw",
            "lidar_topic_name4": "/ns5/points_raw",
            "fusion_enable": fusion_enable,
            "radar_topic": "/sensorRawData",
            "yolo_topic": "/perception/yolo_boxes",
            "camera_config": os.path.join(cfg_dir, "camera.json"),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("fusion_enable", default_value="true"),
        radar_adapter,
        sim_sensors,
        perception,
    ])
