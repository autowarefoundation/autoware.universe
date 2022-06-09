std_msgs/Header header

float64 ego_acc
float64 ego_brake
float64 ego_steer

geometry_msgs/Quaternion orientation

geometry_msgs/Vector3 linear_acceleration

geometry_msgs/Vector3 angular_velocity

int32 num_of_npcs
int32 num_of_pedestrian
int32 num_of_obstacle

ObjectStatus[] npc_list 

ObjectStatus[] pedestrian_list

ObjectStatus[] obstacle_list 