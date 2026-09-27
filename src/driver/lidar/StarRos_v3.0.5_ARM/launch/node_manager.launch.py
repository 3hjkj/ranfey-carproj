from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取 share 目录下的路径
    pkg_share = get_package_share_directory("rfans_driver")

    cfg_path = os.path.join(pkg_share, "revise", "revise.ini")
    # 用 Rviz2_live_cfg.rviz（RViz2 格式，话题 /rfans_points、Fixed Frame rflink）。
    # 原来指的 single_Rviz_cfg.rviz 是 RViz1 格式、话题写死 /points_raw，
    # 在 Humble 的 rviz2 里加载不出点云。
    rviz_config_path = os.path.join(pkg_share, "Rviz2_live_cfg.rviz")

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
                    # —— 点云 UDP 转发 ——
                    # 车机（ARM 原生）：保持 False。厂商库原生匹配、没有 qemu，
                    #   节点直接 publish /rfans_points 就能被 rviz2 发现。
                    # PC（qemu 模拟 aarch64）：设 True，点云会被 UDP 单播到
                    #   net_host:net_port，由宿主上原生的 rfans_relay.py 重新
                    #   publish —— 因为 qemu 下 DDS 的组播发现走不通。
                    # 目标务必写 127.0.0.1 或宿主的真实 IP，别写别的回环别名，
                    # 原因见 rfans_driver.cpp 里 netFrame() 关于 1452 的注释。
                    {"net_forward": False},
                    {"net_host": "127.0.0.1"},
                    {"net_port": 7500},
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
