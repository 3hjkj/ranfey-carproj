# perception/launch/perception.launch.py
# -*- coding: utf-8 -*-
"""
ROS 2 版 perception 启动文件
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    #
    # ─── 参数路径 ───────────────────────────────────────────────
    #
    share_dir = get_package_share_directory("perception")
    cfg_dir = os.path.join(share_dir, "config_json")
    log_dir = os.path.join(share_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)  # 确保日志目录存在

    # 默认参数
    default_params = {
        "cell_config": os.path.join(cfg_dir, "cell.json"),
        "obj_config": os.path.join(cfg_dir, "objs.json"),
        "preprocess_config": os.path.join(cfg_dir, "preprocess.json"),
        "method": 1,  # int
        "topic_name": "fusion",
        "log_path": os.path.join(log_dir, "perception.log"),
        "lidar_topic_name0": "/points_raw",
        "lidar_topic_name1": "/ns2/points_raw",
        "lidar_topic_name2": "/ns3/points_raw",
        "lidar_topic_name3": "/ns4/points_raw",
        "lidar_topic_name4": "/ns5/points_raw",
        # ---- 三传感器融合 ----
        "fusion_enable": False,              # 开启后 /perception/lidar_cells 改为发融合栅格
        "radar_topic": "/sensorRawData",     # 毫米波雷达目标（radar_adapter 发布）
        "yolo_topic": "/perception/yolo_boxes",  # YOLO 检测框
        "camera_config": os.path.join(cfg_dir, "camera.json"),  # 相机粗标定
    }

    #
    # ─── perception 节点 ────────────────────────────────────────
    #
    perception_node = Node(
        package="perception",
        executable="perception",  # 对应 CMakeLists.txt 里的 add_executable 名
        name="perception",
        output="screen",
        parameters=[default_params],
        # respawn=True,                   # 若需要自动重启，则取消注释
        emulate_tty=True,  # 让颜色日志在终端正确显示
    )

    #
    # ─── LaunchDescription 返回 ────────────────────────────────
    #
    return LaunchDescription([perception_node])
