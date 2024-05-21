//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//      该文档的目标是实现ROS的消息回调，同时将主题代码与ROS框架做隔离的中间层
//*****************************************************************

#ifndef INTERFACE_LAT_H
#define INTERFACE_LAT_H



#include "pure_pursuit.h"
#include "Controller.h"
#include "VehicleStat.h"
#include "Planning.h"


/**
 * @brief ros_veh_para_init
 */
void ros_veh_para_init();

/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(char *msg);
/**
 * @brief ref_path_callback
 * @param msg
 */
void ref_path_callback(char *msg);

void calculate_angle();


#endif //ROS_INTERFACE_LAT_H
