//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//*****************************************************************

#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <vector>
#include <cmath>
#include "Eigen/Core"

#include <iostream>
using namespace std;

//--数据结构 车辆参数
struct VehPara{
  float len;                    //车长
  float width;                  //车宽
  float hight;                  //车高
  float mass;                   //车重
  float f_tread;                //前轮轮距
  float r_tread;                //后轮轮距
  float wheelbase;              //轴距
  float steering_ratio;         //转向比
  float max_wheel_angle;        //最大车轮转角
  float max_steer_angle;        //最大方向盘转角
  float wheel_diam;             //车轮直径
};

/**
 * @brief pure_veh_para_init 从参数服务器中获取车辆数模参数
 * @param arg
 */
void pure_veh_para_init(const struct VehPara &arg);

/**
 * @brief pure_set_veh_speed  接收处理订阅的车速信息
 * @param msg_speed
 */
void pure_set_veh_speed(const float arg);

/**
 * @brief pure_set_ref_path 接收处理订阅的跟随路径
 * @param x
 * @param y
 */
void pure_set_ref_path(const std::vector<float> &x,const std::vector<float> &y);

/**
 * @brief pure_pursuit  纯跟踪算法
 * @return
 */
float pure_pursuit();

uint8_t set_turn_light();

#endif //PURE_PURSUIT_H



