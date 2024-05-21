//*****************************************************************
// 作者 ：杨东
// 时间 ：2019.04.15
// 版权 ：重庆邮电大学自动化学院汽车电子工程中心智能车辆技术团队
// 说明 ：
//*****************************************************************
#ifndef INTERFACE_LON_H
#define INTERFACE_LON_H


#include "lon_controller.h"
//车身状态
#include "VehicleStat.h"
#include "Localization.h"

//停车、AEB、正常行驶
#include "Planning.h"

//-->>控制消息发布
#include "Controller.h"
//#include "custom_msgs/TrqBreCmd485.h"

#include <cstring>




//-->>消息回调
/**
 * @brief veh_status_callback
 * @param msg
 */
void veh_status_callback(char *msg);

/**
 * @brief veh_pose_callback
 * @param msg
 */
void veh_pose_callback(char *msg);

/**
 * @brief run_req_callback
 * @param msg
 */
void run_req_callback(char *msg);

#endif //INTERFACE_LON_H
