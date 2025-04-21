# localization
Xyz[] wheel_speed_left
Xyz[] wheel_speed_right
Xyz xy # 平面坐标 z-航向
Xyz imu
Xyz speed  # 807->327  东北天速度
float64 abs_speed  # 807->327 车辆速度
float64 car_speed  #  底层车辆速度
Xyz car_line_accelerate # 809->329 车辆坐标系Xyz线加速度
Xyz car_ang_speed  # 812->32c 车辆坐标系Xyz角速度
int8 satellite_status # 0-无定位 1-单点 2-伪距差分 3-组合推算 4-固定解 5-浮点解
int8 system_state # 0-初始化 1-卫星导航 2-组合导航 3-纯惯导
int8 gps_num_sats_used # 搜星数
PoseAngle pose # 810->32a roll = 横滚角 yaw=航向角 pitch=俯仰角
GPS  gps # 804和805->324 325 维度 经度 高度
Time utm_time # 800->320 gps_time
float64 second_time 
float64 system_time # ros::Time
##slam
bool slam_loc_ok
geometry_msgs/PoseStamped salm_pose
