#ifndef PATHPLANNING_H
#define PATHPLANNING_H

// #include "ros/ros.h"
#include "data_type.h"
// #include <fstream>
// #include <sstream>
#include "frenet.h"
// #include <iostream>
#include "spline.h"
// #include <cstdlib>
// #include "compute/map.h"
// #include <custom_msgs/TurnLightCmd.h>
// #include <custom_msgs/Ambulance.h>
class PathPlanning
{
public:
    PathPlanning();              //获取参考路径
    ~PathPlanning();
    void generate_path(CurrentPose &curr_pose, LaneLine &lanelines, /*CurrentState &curr_state*/ float velocity);  
    
    vector<double> x_ref;    //规划出的路径点列的x值，地图坐标系下    
    vector<double> y_ref;    //规划出的路径点列的y值
    vector<double> v_ref;    //规划出的路径点列的v值

    vector<double> map_waypoints_x;    //参考路径点列的x值
    vector<double> map_waypoints_y;    //参考路径点列的y值
    vector<double> map_waypoints_s;    //参考路径点列的s值

    
    void Get_Curr_Sta(CurrentPose &curr_pose_temp, /*CurrentState &curr_state*/float velocity);         //获取主车当前状态（位姿和速度）
    void Get_Path_Ref(LaneLine &laneline);                                            //获取参考路径
    void get_plan_dis(float vel_speed_ref);                                          //计算规划长




    void calculate_trajectory();                 //计算车辆行驶轨迹
    void simple_from_spline();                   //采样路径并转化为笛卡尔坐标系x，y

    void push_sd(vector<double> &pts_s,vector<double> &pts_d);        //压入sd


    tk::spline s_path;           //曲线拟合的对象
    double curr_car_s;           //主车定位信息
    double curr_car_d;
    double curr_car_x;
    double curr_car_y;

    float vel_speed_self;        //主车自身行驶速度,km/h
    float plan_distance;         //规划距离
    int first_time = 0;

    bool is_changing_lane = false;     //变道状态
    float offset = 0;            //横向偏移后的位置
    float first_s = 0;            //横向偏移后的位置

};

#endif // PATHPLANNING_H
