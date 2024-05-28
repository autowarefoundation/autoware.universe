//********************************************************************************************
// 作   者-->:  杨 东
// 创建时间-->:  2019.08.05
// 修改时间-->:  2019.08.05
// 版   权-->:  重庆邮电大学\自动化学院\汽车电子工程中心\智能汽车技术研究所
//--------------------------------------------------------------------------------------------
// 文档说明 ：
//**********************************************************************************************

#ifndef V_CHASSIS_INTERFACE_H
#define V_CHASSIS_INTERFACE_H

#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <can_protocol.h>
#include "socketcan.h"
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define NO_LOCAL_DEBUG 1

extern int brake_flag;
extern int brake_count;

struct VehicleStat{
  float    VehicleSpeed;                //车速
  float    SAS_SteeringAngle;           //方向盘转角速度
  float    EMS_EngineSpeed;             //发动机转速
  float    EMS_EngineThrottlePosition;  //发动机节气门位置
  float    EMS_AccPedal;                //加速踏板位置
  uint8_t  EMS_BrakePedalStatus;        //制动踏板状态
  uint8_t  TCU_GearShiftPositon;        //换挡器位置
  uint8_t  autoDrivetoRemote;

};

struct Control_Mode {
  uint8_t      start;                     //qidong
  uint8_t      mode_type;                 //模式类型  00：无， 自动模式：（11：自动生成地图，12：已有地图） 遥控模式：20
  uint8_t      enable;                    //紧急使能  
  uint8_t      brake_enable;              //制动使能  0：制动，1：可行驶
  float        speed_require;             //速度要求
  float        angle_require;             //角度要求
  uint8_t      RemoteToAutoDrive;
};

struct Trq_Bre_Cmd{
  uint8_t  bre_enable;
  float    bre_value;
  uint8_t  trq_enable;
  float    trq_value;
};

struct RtkImuState
{
  float pitch;            
  float roll;         
  float heading;        
  float speed2d;      //--定位下的车辆速度 m/s
};
extern int can_fd;

/**
 * @brief start_status_monitor_thread 开启监控线程
 * @return
 */
pthread_t start_status_monitor_thread();

//pthread_t start_manual_auto_driving_thread();

// /**
//  * @brief start_send_timer  开启命令发送定时器
//  * @param t_ms 定时器设定时间
//  */
void start_send_timer(int t_ms);
void vehicle_ready();

void get_veh_status(struct VehicleStat &stutus);
//------------------------------------------------->>消息回调接口------------------------------------------------
void set_control_mode(struct Control_Mode &msg);
void set_trq_bre_cmd(struct Trq_Bre_Cmd &msg);
void set_steering_cmd(const float angle);
void set_TurnLight_cmd(const uint8_t TurnLight);
void set_rtk_imu_state(struct RtkImuState &arg);


// //---------------------------------------------------------------------------------------------------------------

#endif // V_CHASSIS_INTERFACE_H

