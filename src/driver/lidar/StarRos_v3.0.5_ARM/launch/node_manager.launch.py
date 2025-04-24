from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 share 目录下的路径
    pkg_share = get_package_share_directory("rfans_driver")

    cfg_path = os.path.join(pkg_share, "revise", "revise.ini")
    rviz_config_path = os.path.join(pkg_share, "single_Rviz_cfg.rviz")

    return LaunchDescription(
        [
            Node(
                package="rfans_driver",
                executable="driver_node",
                name="rfans_driver",
                output="screen",
                parameters=[
                    {"frame_id": "rflink"},
                    {"RT": "0,0,180,0,0,0,1"},
                    {"model": "R-Fans-32"},
                    {"display_mode": "overlay"},
                    {"device_ip": "192.168.5.3"},
                    {"device_port": 2014},
                    {"heart_port": 2030},
                    {"msg_port": 2015},
                    {"rps": 10},
                    {"readfile_path": ""},
                    {"cfg_path": cfg_path},
                    {"filter_path": ""},
                    {"save_xyz": False},
                    {"OutXYZ_path": "/home/bkth/ros/rfans16/ros.xyz"},
                    {"save_isf": False},
                    {"OutISF_path": "/home/bkth/ros/rfans16"},
                    {"use_double_echo": False},
                    {"use_gps": False},
                    {"read_once": False},
                    {"algorithm_default": False},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                output="screen",
                arguments=["-d", rviz_config_path],
            ),
        ]
    )
