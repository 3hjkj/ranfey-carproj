"""启动毫米波雷达驱动节点。

    ros2 launch radar_gcan radar_gcan.launch.py

注意：本节点往 /sensorRawData 发真雷达目标，radar_adapter 的 sim 模式往同一个
话题发假目标，**不要同时跑**（节点启动时会检查并报错）。

标定清单（仓库里没有 .dbc/.arxml，int.h 也没有 factor/offset，只能实测反推）：

  1. 先用 publish_uncalibrated:=true 起，看 /radar_gcan/stats 里各槽位的原始值，
     或直接 ros2 launch radar_gcan radar_gcan.launch.py publish_uncalibrated:=true
  2. 把靶标摆到卷尺量好的已知距离 L 米、横向偏移 Y 米处，读原始 l_long / l_lat：
         factor_long_m    = L / l_long
         factor_lat_m     = Y / l_lat
  3. 速度系数同理，让靶标以已知速度接近，读 v_long：factor_vlong_mps = v / v_long
  4. **存疑，务必实测**：雷达同时接收整车偏航（0x130）和车速（0x3E9），说明它可能
     输出的是**绝对速度**而不是相对速度。这决定了 vx 该怎么进卡尔曼 ——
     按相对速度用会让滤波器把自车速度算成目标速度，等于凭空造出一个高速目标。
     验证方法：车静止，对着路边完全静止的杆子看 vx 是否≈0；若 vx≈-自车速度，
     那它给的就是绝对速度，需要在下游减掉自车速度才能当相对速度用。
  5. 填完系数后 max_range_m 的门限才会生效，它是系数填错数量级时的兜底报警

未标定时节点只解码、不发布（x 和 vy 直接进 decision_planning 的紧急停车判据，
填错系数的距离值比不发更危险）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ── 换算系数：原始计数 × factor = 物理量 ──────────────────────────────
# 留 0 = 未标定。这四个是必填项（x 和 vy 直接进 decision_planning 的停车判据）。
FACTORS = {
    'factor_long_m': 0.0,       # L_LongObj -> 米（纵向距离）
    'factor_lat_m': 0.0,        # L_LatObj  -> 米（横向距离）
    'factor_vlong_mps': 0.0,    # V_LongObj -> m/s（纵向速度）
    'factor_vlat_mps': 0.0,     # V_LatObj  -> m/s（横向速度）
}

PARAMS = {
    'host': '192.168.5.137',    # 广成 GCAN-202 转换机
    # 实测端口映射(2026-09-26, SN 21102914):
    #   4001 = CAN1   9998 = CAN2   22080 = 配置口
    # 手册写的 4002/8001 在这台设备上并未开放 —— 别照抄手册。
    'port': 4001,
    'publish_hz': 20.0,
    'stale_s': 0.5,             # 超过这个时间没更新的槽位不再发布
    # 装反了会导致目标左右镜像；真车对标发现左右颠倒就改成 -1.0
    'lateral_sign': 1.0,
    # 雷达在车上的安装位置（米）：装在车头前 0.5 m 就填 0.5
    'offset_x': 0.0,
    'offset_y': 0.0,
    'max_range_m': 300.0,       # 标定后的合理性门限，未标定时不生效
    'warmup_s': 0.5,            # 每次连上后先丢这么久，防转换机吐历史缓冲
    'stats_period_s': 1.0,      # 0 = 不发 /radar_gcan/stats
    **FACTORS,
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_uncalibrated', default_value='false',
            description='未标定时也发布（原始计数当米/m·s⁻¹ 直接发）。'
                        '仅供台架跑通链路，实车别开'),
        DeclareLaunchArgument(
            'host', default_value=PARAMS['host'],
            description='GCAN 转换机 IP'),
        Node(
            package='radar_gcan',
            executable='radar_gcan_node.py',
            name='radar_gcan',
            output='screen',
            emulate_tty=True,
            parameters=[PARAMS, {
                # 必须包 ParameterValue(value_type=bool)：LaunchConfiguration 出来
                # 的是字符串 'true'，而节点那边按 bool 声明，类型对不上会直接抛
                # InvalidParameterTypeException，节点起不来。
                'publish_uncalibrated': ParameterValue(
                    LaunchConfiguration('publish_uncalibrated'),
                    value_type=bool),
                'host': LaunchConfiguration('host'),
            }],
        ),
    ])
