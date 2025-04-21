## mqtt_recv_initaldata
int8 obj1_id
int8 obj1_type
float64 obj1_lat
float64 obj1_lon
float64 obj1_speed 
float64 obj1_heading

int8 obj2_id
int8 obj2_type
float64 obj2_lat
float64 obj2_lon
float64 obj2_speed 
float64 obj2_heading



## can_com_pub publish vehicle & mqtt_data
float32 vehicle_x
float32 vehicle_y
float32 vehicle_speed
float32 vehicle_heading
float32 pre_aiming_x
float32 pre_aiming_y

float32 v_obj_pointx1                   
float32 v_obj_pointx2   
float32 v_obj_pointy1
float32 v_obj_pointy2

float64 obj1_w842_utm_x
float64 obj1_w842_utm_y
float64 obj1_w842_utm_speed
float64 obj1_w842_utm_heading
float64 obj1_w842_utm_id
float64 obj1_w842_utm_type

float64 obj2_w842_utm_x
float64 obj2_w842_utm_y
float64 obj2_w842_utm_speed
float64 obj2_w842_utm_heading
float64 obj2_w842_utm_id
float64 obj2_w842_utm_type



## python publish coordinate transform roadsideobj_data
float64 target1_l_lat
float64 target1_l_lon
float64 target1_v_lat
float64 target1_v_lon
int8 target1_id
int8 target1_type

float64 target2_l_lat
float64 target2_l_lon
float64 target2_v_lat
float64 target2_v_lon
int8 target2_id
int8 target2_type