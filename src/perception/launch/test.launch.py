# perception/launch/perception.launch.py
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:
    share_dir = get_package_share_directory('perception')
    cfg_dir   = os.path.join(share_dir, 'config_json')
    log_dir   = os.path.join(share_dir, 'log')
    os.makedirs(log_dir, exist_ok=True)

    vis_node = Node(
        package='perception_visualization',
        executable='vis',
        name='vis',
        output='screen'
    )

    perception_params = {
        'cell_config'      : os.path.join(cfg_dir, 'cell.json'),
        'obj_config'       : os.path.join(cfg_dir, 'objs.json'),
        'preprocess_config': os.path.join(cfg_dir, 'preprocess.json'),
        'method'           : 1,
        'topic_name'       : 'fusion',
        'log_path'         : log_dir + '/',
        'lidar_topic_name0': '/rfans_driver/rfans_points',
        'lidar_topic_name1': '/ns2/lidar_points',
        'lidar_topic_name2': '/ns3/lidar_points',
        'lidar_topic_name3': '/ns4/lidar_points',
        'lidar_topic_name4': '/ns5/lidar_points',
    }

    perception_node = Node(
        package='perception',
        executable='perception',
        name='perception',
        output='screen',
        parameters=[perception_params],
        emulate_tty=True        # 需要彩色日志时保留
        # respawn=True, respawn_delay=2.0  # 若要掉进程重启
    )

    return LaunchDescription([
        vis_node,
        perception_node
    ])
