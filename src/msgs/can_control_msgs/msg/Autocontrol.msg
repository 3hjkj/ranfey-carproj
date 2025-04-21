std_msgs/Header header 
# rgb light
int8 left_traffic_light   #0 green  ,1  red,2 yellow,3 none
int8 middle_traffic_light #0 green  ,1  red,2 yellow,3 none
int8 right_traffic_light #0 green  ,1  red,2 yellow,3 none
float64 target_back_obj_distance
## Front object
float64 front_v_long_obj
float64 front_l_long_obj
float64 front_v_lat_obj
float64 front_l_lat_obj
float64 front_width
uint8 front_class
float64 front_a_long_obj
float64 front_a_lat_obj
float64 front_length
float64 front_height

## Class of detected object:
# 0-->Undetermined
# 1-->Vehicle
# 2-->Motorcycle
# 3-->Truck
# 4-->Pedestrian
# 5-->pole
# 6-->Tree
# 7-->Animal
# 8-->Gen OnRoad Obj
# 9-->Bicycle
# 10-->Undentified Obj
##end


## Left object
float64 left_v_long_obj
float64 left_l_long_obj
float64 left_v_lat_obj
float64 left_l_lat_obj
float64 left_width
uint8 left_class
float64 left_a_long_obj
float64 left_a_lat_obj

float64 left_length
float64 left_height

## Leftfront object
float64 leftfront_v_long_obj
float64 leftfront_l_long_obj
float64 leftfront_v_lat_obj
float64 leftfront_l_lat_obj
float64 leftfront_width
uint8 leftfront_class
float64 leftfront_a_long_obj
float64 leftfront_a_lat_obj

float64 leftfront_length
float64 leftfront_height

## Right object
float64 right_v_long_obj
float64 right_l_long_obj
float64 right_v_lat_obj
float64 right_l_lat_obj
float64 right_width
uint8 right_class
float64 right_a_long_obj
float64 right_a_lat_obj

float64 right_length
float64 right_height

## Rightfront object
float64 rightfront_v_long_obj
float64 rightfront_l_long_obj
float64 rightfront_v_lat_obj
float64 rightfront_l_lat_obj
float64 rightfront_width
uint8 rightfront_class
float64 rightfront_a_long_obj
float64 rightfront_a_lat_obj

float64 rightfront_length
float64 rightfront_height

## Back object
float64 back_v_long_obj
float64 back_l_long_obj
float64 back_v_lat_obj
float64 back_l_lat_obj
float64 back_width
uint8 back_class
float64 back_a_long_obj
float64 back_a_lat_obj

float64 back_length
float64 back_height

## Rightback object1
float64 rightback_v_long_obj
float64 rightback_l_long_obj
float64 rightback_v_lat_obj
float64 rightback_l_lat_obj
float64 rightback_width
uint8 rightback_class
float64 rightback_a_long_obj
float64 rightback_a_lat_obj

float64 rightback_length
float64 rightback_height

## Leftback object2
float64 leftback_v_long_obj
float64 leftback_l_long_obj
float64 leftback_v_lat_obj
float64 leftback_l_lat_obj
float64 leftback_width
uint8 leftback_class
float64 leftback_a_long_obj
float64 leftback_a_lat_obj

float64 leftback_length
float64 leftback_height

## RT1 object
float64 rt1_v_long_obj
float64 rt1_l_long_obj
float64 rt1_v_lat_obj
float64 rt1_l_lat_obj
float64 rt1_width
uint8 rt1_class
float64 rt1_a_long_obj
float64 rt1_a_lat_obj
uint16 fusion_count

float64 rt1_length
float64 rt1_height
