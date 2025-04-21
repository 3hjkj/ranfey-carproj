int16 vfpoint
float32 aim_x
float32 aim_y
float32 speed

bool traffic_light_stop  ##红绿灯行驶判断  true  停止   false 行驶
bool trace_stop##路径行驶判断  true  停车   false 行驶

bool traffic_light_driving  ##红绿灯行驶判断  true  停止   false 行驶
bool trace_driving##路径行驶判断  true  停车   false 行驶
int16 loc_type  #0 gps  ,1 slam  ,-1 None
float32 loca_x
float32 loca_y
float32 loca_yaw
float32 loca_pitch
int8 start_autoparking