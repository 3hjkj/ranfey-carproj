int64 idx
float32 rel_x # car and obj 
float32 rel_y
float32 rel_z
float32 abs_x # ground and obj
float32 abs_y
float32 abs_z
float32 rel_vx # car and obj speed
float32 rel_vy
float32 rel_vz
float32 rel_ax # car and obj speed
float32 rel_ay
float32 rel_az
float32 abs_vx # ground and obj
float32 abs_vy
float32 abs_vz
float32 length
float32 width
float32 height
float32 confidence 
float32 speed
float32 rel_heading
float32 abs_heading
int32 type  # 0 - unknown; 1 - truck; 2 - car; 3 - person ; 4 - cyclist  
int32 source # 0 -unknown ; 1 - lidar; 2 - radar ; 3 - camera ; 4 - ultrasound
float64 time
float32 score
bool used
