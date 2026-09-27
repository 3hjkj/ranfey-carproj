"""PC 专用：在 qemu 容器里跑驱动，并把点云 UDP 转发给宿主。

车机上**不要用这个**，用 node_manager.launch.py 就行 —— 那里没有 qemu，
节点直接 publish 就能被 rviz2 发现，多一跳转发是累赘。

为什么需要转发：容器里跑的是 aarch64 二进制（厂商库只有 ARM 版），
靠 qemu-user 模拟；qemu 对组播 socket 选项支持不全，DDS 的 SDP 发现
走不通，宿主的 rviz2 发现不了这个节点。

用法（在容器内）：
    ros2 launch rfans_driver pc_relay.launch.py

然后**在宿主上**另开一个终端跑中继。注意这里不能用 `ros2 run`：
本包只在容器的 aarch64 环境里构建过（厂商库只有 ARM 版），宿主上并没有
安装这个包。中继脚本本身是纯 Python，直接跑就行：

    source /opt/ros/humble/setup.bash
    <仓库>/src/driver/lidar/StarRos_v3.0.5_ARM/scripts/rfans_relay.py \
        --ros-args -p frame_id:=rflink

之后宿主上的 rviz2 / ros2 topic hz 就都能用了。
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("rfans_driver")
    cfg_path = os.path.join(pkg_share, "revise", "revise.ini")

    return LaunchDescription(
        [
            Node(
                package="rfans_driver",
                executable="driver_node",
                name="rfans_driver",
                output="screen",
                parameters=[
                    # 这两个必须和宿主 relay 的 --ros-args -p frame_id 一致，
                    # 否则 rviz2 里两条路径（直连 / 经 relay）的坐标系对不上
                    {"frame_id": "rflink"},
                    {"RT": "0,0,180,0,0,0,1"},
                    {"model": "R-Fans-32"},
                    {"display_mode": "overlay"},
                    {"device_ip": "192.168.5.3"},
                    {"device_port": 2014},
                    {"heart_port": 2030},
                    {"msg_port": 2015},
                    {"rps": 10},
                    {"cfg_path": cfg_path},
                    {"filter_path": ""},
                    # 这条不能省！readfile_path 非空 = 回放模式，而驱动的参数
                    # 默认值是个写死的 ISF 路径（/home/bkth/cfans128/...）。
                    # 不显式清空的话，驱动会去回放那个不存在的文件：日志刷
                    # deviceID=0、一个点都不出，最后段错误。它只认这一个开关
                    # 判断实时还是回放（见 rfans_driver.cpp 的 simu_filepath）。
                    {"readfile_path": ""},
                    # 不开存盘，纯转发；要落盘另加 save_bin / OutBIN_path
                    {"save_xyz": False},
                    {"save_bin": False},
                    {"save_isf": False},
                    {"use_double_echo": False},
                    {"use_gps": False},
                    {"read_once": False},
                    {"algorithm_default": False},
                    # —— 转发开关 ——
                    # 容器是 --network host，和宿主共享网络栈，所以 127.0.0.1
                    # 直接就是宿主。千万别改成别的地址，见 netFrame() 注释。
                    {"net_forward": True},
                    {"net_host": "127.0.0.1"},
                    {"net_port": 7500},
                ],
            ),
        ]
    )
