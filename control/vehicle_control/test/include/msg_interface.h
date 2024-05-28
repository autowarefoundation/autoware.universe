#ifndef MSG_INTERFACE_H
#define MSG_INTERFACE_H

#include "v_chassis_interface.h"

#include "Controller.h"          
#include "ControlMode.h"
#include "Localization.h"
#include "Planning.h"          
#include "VehicleStat.h"

void veh_pose();

//行驶控制接口
void control_mode_callback(char *msg);
void steering_cmd_callback(char *msg);
void trq_bre_cmd_callback(char *msg);
//bool get_TurnLight_cmd_callback( custom_msgs::TurnLightCmd::Request &Req, custom_msgs::TurnLightCmd::Response &Res);
//void self_mode_callback(const custom_msgs::Request::ConstPtr &msg);
#endif // ROS_MSG_INTERFACE_H
