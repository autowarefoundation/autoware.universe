#ifndef NODE_ROUTING_CORE_H
#define NODE_ROUTING_CORE_H

#include"data_type.h"
#include "frenet.h"
// #include "frenet.h"
// #include "custom_msgs/ObjectArray.h"
// #include "custom_msgs/LaneLineArray.h"
// #include "custom_msgs/NaviData.h"
// #include "custom_msgs/RoadAttri.h"
// #include "custom_msgs/Request.h"
// #include "custom_msgs/Control.h"
// #include "custom_msgs/Route.h"
// #include "custom_msgs/AEB.h"
// #include <custom_msgs/VehicleStat.h>
// #include <geometry_msgs/Pose2D.h>
// #include <nav_msgs/Path.h>
// #include <visualization_msgs/Marker.h>
#include "Planning.h"
// #include <custom_msgs/CurPose.h>
#include "pathplanning.h"
#include "Localization.h"



    // ros::ServiceServer routing_service;  //路径规划服务
    // ros::ServiceServer control_service;  //控制接口服务

    // //主车位姿、车道线信息
    // CurrentPose current_pose;     //主车当前位姿
    // CurrentPose current_pose_temp;
    // CurrentState navi_data;       //主车当前状态（来自惯导的速度）
    // bool is_Parking;   //判断前进后退

    // //
    // double Expedspeed;            //期望速度
    // ChangeLane change_lane_info;  //变道信息

    // AEB_STOP abe_stop;
    // Speed_task speed_task;
    // Backcar_task backcar_task;
    // map<string,AEB_STOP>  AEB_list; 
    // map<string,AEB_STOP>  STOP_list; 
    // map<string,AEB_STOP>::iterator iter;


    // //发布出去的变量
    // custom_msgs::Path path;            //待发布的路径   
    // nav_msgs::Path path_vis;
    // visualization_msgs::Marker marker; //显示参考路径
    // custom_msgs::AEB stop_msg;         //停车消息
    // custom_msgs::Request request;        

    void Line_Recv_callback(char *msg);   //获取车道线
    // void onCurrentPoseMsgRecvd(const custom_msgs::CurPose::ConstPtr &msg);           //获取主车当前位姿
    // void onNaviDataRecvd(const custom_msgs::NaviData::ConstPtr &msg);                //获取当前惯导数据
    void toPath(Path_h &path_msg, CurrentPose &current_pose_temp); 
    void Current_Pose_callback(void *dora_context, char *msg);
    // void toPathVis(const custom_msgs::Path &src,nav_msgs::Path &dst);
    // void onRoadSpeedMsgRecvd(const custom_msgs::RoadAttri::ConstPtr &msg);    //速度接收与处理
    // bool onRoutingHandleFunction(custom_msgs::Route::Request &req,custom_msgs::Route::Response &res);   //routing_service 处理函数
    // bool onControlHandleFunction(custom_msgs::Control::Request &req,custom_msgs::Control::Response &res);   //control_service 处理函数
    // void onVehicleStatRecvd(const custom_msgs::VehicleStat::ConstPtr &msg); 





#endif // NODE_ROUTING_CORE_H

