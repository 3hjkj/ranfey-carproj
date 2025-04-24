from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # 声明可配置参数
    declared_args = [
        DeclareLaunchArgument("read_fast", default_value="false"),
        DeclareLaunchArgument("read_once", default_value="false"),
        DeclareLaunchArgument("repeat_delay", default_value="0.0"),
        DeclareLaunchArgument("Device_1", default_value="true"),
        DeclareLaunchArgument("Device_2", default_value="true"),
        DeclareLaunchArgument("Device_3", default_value="false"),
        DeclareLaunchArgument("Device_4", default_value="false"),
    ]

    # device 1
    device_1 = GroupAction(
        [
            PushRosNamespace("ns1"),
            Node(
                package="rfans_driver",
                executable="driver_node",
                name="rfans_driver",
                output="screen",
                parameters=[
                    {"frame_id": "world"},
                    {"RT": "0.0,0.0,0.0,0.0,0.0,0.0"},
                    {"model": "R-Fans-16"},
                    {"control_name": "rfans_control"},
                    {"device_ip": "192.168.5.3"},
                    {"device_port": 2014},
                    {"heart_port": 2034},
                    {"msg_port": 2015},
                    {"rps": 10},
                    {"Is_Start": LaunchConfiguration("Device_1")},
                    {"readfile_path": ""},
                    {"cfg_path": "/home/bkth/Desktop/CK128/revise128.ini"},
                    {"use_double_echo": False},
                    {"use_gps": False},
                    {"read_fast": LaunchConfiguration("read_fast")},
                    {"read_once": LaunchConfiguration("read_once")},
                    {"repeat_delay": LaunchConfiguration("repeat_delay")},
                    {"cut_angle_range": 360.0},
                ],
            ),
        ]
    )

    # device 2
    device_2 = GroupAction(
        [
            PushRosNamespace("ns2"),
            Node(
                package="rfans_driver",
                executable="driver_node",
                name="rfans_driver",
                output="screen",
                parameters=[
                    {"frame_id": "world"},
                    {"RT": "0.0,0.0,0.0,0.0,0.0,0.0"},
                    {"model": "C-Fans-128"},
                    {"control_name": "rfans_control"},
                    {"device_ip": "192.168.0.3"},
                    {"device_port": 2014},
                    {"heart_port": 2030},
                    {"msg_port": 2015},
                    {"rps": 10},
                    {"Is_Start": LaunchConfiguration("Device_2")},
                    {"readfile_path": ""},
                    {"cfg_path": "/home/bkth/Desktop/CK128/revise128.ini"},
                    {"use_double_echo": False},
                    {"use_gps": False},
                    {"read_fast": LaunchConfiguration("read_fast")},
                    {"read_once": LaunchConfiguration("read_once")},
                    {"repeat_delay": LaunchConfiguration("repeat_delay")},
                    {"cut_angle_range": 360.0},
                ],
            ),
        ]
    )

    # u_coordinate node
    u_coordinate = Node(
        package="rfans_driver",
        executable="u_coordinate",
        name="u_coordinate",
        output="screen",
    )

    # 可选 RViz 启动
    # from ament_index_python.packages import get_package_share_directory
    # import os
    # rviz_config = os.path.join(get_package_share_directory('rfans_driver'), 'multi_Rviz_cfg.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz',
    #     output='screen',
    #     arguments=['-d', rviz_config]
    # )

    return LaunchDescription(
        declared_args
        + [
            device_1,
            device_2,
            u_coordinate,
            # rviz_node  # 如果需要，取消注释即可
        ]
    )
