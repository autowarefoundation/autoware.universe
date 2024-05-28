//********************************************************************************************
//新小底盘车（pix普朗克新版）
//**********************************************************************************************

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cstring>
#include <cstdint>

//-------------------------------------------------------------控制报文-------------------------------------------------------------------------------

//制动控制报文
struct MSG_ID131 {
    uint8_t ACU_ChassisBrakeEn; //车辆刹车控制使能 "0:disable 1:enable"
    uint8_t ACU_ChassisAebCtrl; //AEB使能（预留，自动驾驶实现） "0:disable 1:enable"
    float ACU_ChassisBrakePdlTarget;  //车辆刹车控制 0-100% 精度0.1
    uint8_t ACU_ChassisEpbCtrl; //驻车控制 "0:default 1:brake 2:release"
    // uint8_t ACU_BrakeLifeSig;   //循环计数0~15
    // uint8_t ACU_CheckSum_131;   //校验sum=byte0 xor byte1 xor...byte6
};

//驱动控制报文
struct MSG_ID130 {
    uint8_t ACU_ChassisDriverEnCtrl;//车辆加速控制使能 "0:disable 1:enable"
    uint8_t ACU_ChassisDriverModeCtrl;  //驱动模式控制 "0:speed ctrl mode / 1:throttle ctrl mode / 2:reserve / 3:reserve"
    uint8_t ACU_ChassisGearCtrl;    //档位控制 "0:default N / 1:D / 2:N / 3:R"
    float ACU_ChassisSpeedCtrl;   //车辆速度控制 0-50 精度 0.01 m/s
    float ACU_ChassisThrottlePdlTarget; //车辆油门控制 0-100%	精度0.1
    // uint8_t ACU_DriveLifeSig;   //循环计数0~15
    // uint8_t ACU_CheckSum_130;   //校验sum=byte0 xor byte1 xor...byte6
};

//转向控制报文
struct MSG_ID132 {
    uint8_t ACU_ChassisSteerEnCtrl; //车辆转向控制使能 "0:disable 1:enable"
    uint8_t ACU_ChassisSteerModeCtrl;   //转向模式控制 "0:front ackerman / 1:same front and back / 2:front different back / 3:back ackrman / 4:front back"
    int16_t ACU_ChassisSteerAngleTarget;  //车辆转向控制（前） -500 - +500
    int16_t ACU_ChassisSteerAngleRearTarget;  //车辆转向控制（后） -500 - +500
    uint8_t ACU_ChassisSteerAngleSpeedCtrl;   //转向角速度 
    // uint8_t ACU_SteerLifeSig;   //循环计数0~15
    // uint8_t ACU_CheckSum_132;   //校验sum=byte0 xor byte1 xor...byte6
};

//车身控制1
struct MSG_ID133 {
    uint8_t ACU_VehicleLeftLampCtrl;    //左转向灯控制 "0:off / 1:on"
    uint8_t ACU_VehicleRightLampCtrl;   //右转向灯控制 "0:off / 1:on"
    uint8_t ACU_CheckSumEn; //校验模式使能(预留) "0:enable / 1:disable"
};








//--------------------------------------状态报文----------------------------------------------------

//车辆制动状态反馈
struct MSG_ID531 {
    uint8_t VCU_ChassisBrakeEnSta;  //制动使能状态 "0:disable / 1:enable"
    uint8_t VCU_VehicleBrakeLampFb; //制动灯状态反馈 "0:off / 1:on"
    uint8_t VCU_ChassisEpbFb;   //驻车状态 "0:release / 1:brake / 2:releasing / 3:braking"
    uint16_t VCU_ChassisBrakePadlFb; //制动实际反馈 制动值反馈0~100%
    uint8_t VCU_AebEnStaFb; //AEB开启状态反馈 "0:off / 1:on"
    uint8_t VCU_AebTriggerStaFb;    //AEB触发状态反馈 "0:off / 1:aeb trigger"
};

//车轮转速反馈
struct MSG_ID539 {
    uint16_t VCU_ChassisWheelRpmLf; //左前轮转速 -2000 - +2000
    uint16_t VCU_ChassisWheelRpmRf; //右前轮转速 -2000 - +2000
    uint16_t VCU_ChassisWheelRpmLr; //左后轮转速 -2000 - +2000
    uint16_t VCU_ChassisWheelRpmRr; //右后轮转速 -2000 - +2000
};

//车辆驱动状态反馈
struct MSG_ID530 {
    uint8_t VCU_ChassisDriverEnSta; //驱动使能状态 "0x0:disable / 0x1:enable"
    uint8_t VCU_ChassisDiverSlopover;   //驱动控制越界提醒 "0:normal / 1:over slop"
    uint8_t VCU_ChassisDriverModeSta;   //驱动模式反馈 "0:speed ctrl mode / 1:throttle ctrl mode / 2:reserve / 3:reserve"
    uint8_t VCU_ChassisGearFb;  //档位状态 "0:no use / 1:D / 2:N / 3:R"
    float VCU_ChassisSpeedFb; //车速实际反馈 反馈与实际速度有关
    float VCU_ChassisThrottlePaldFb;    //油门请求值反馈 油门请求值0~100%
    float VCU_ChassisAccelerationFb;    //车辆加速度 预留
};

//车辆动力电源状态反馈
struct MSG_ID535 {
    uint8_t VCU_ChassisBmsReserved_1;   //预留
    uint8_t VCU_ChassisPowerSocFb;  //车辆动力电池电量 0-100%
    float VCU_ChassisPowerVoltFb;   //车辆动力电池电压 0-1000V 精度0.1
    float VCU_ChassisPowerCurrFb;   //车辆动力电池电流 -1000 - 1000A 精度0.1
    uint8_t VCU_ChassisBmsMaxTemp;  //BMS最高单体温度 -40 - 80度
    uint8_t VCU_ChassisBmsReserved_2;   //预留
};

//车辆转向状态反馈
struct MSG_ID532 {
    uint8_t VCU_ChassisSteerEnSta;  //转向使能状态 "0:disable / 1:enable"
    uint8_t VCU_ChassisSteerSlopover;   //转向控制越界提醒 "0:normal / 1:over slop"
    uint8_t VCU_ChassisSteerWorkMode;   //转向线控模式反馈 "0:machine / 1:wire / 2:power"
    uint8_t VCU_ChassisSteerModeFb; //转向模式反馈 "0:front ackerman / 1:same front and back / 2:front different back / 3:back ackrman / 4:front back"
    int16_t VCU_ChassisSteerAngleFb;  //前转向方向盘转角反馈 最大最小值与实际底盘转向系统参数有关
    int16_t VCU_ChassisSteerAngleRearFb;  //后转向方向盘转角反馈 最大最小值与实际底盘转向系统参数有关
};

//车辆警告和故障
struct MSG_ID537 {
    uint8_t VCU_SysUnderVolt;   //蓄电池电压过低 "0:normal / 1:under volt"
    uint8_t VCU_SysFlt; //系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysBrakeFlt;    //制动系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysSteerFrontFlt;   //前转向系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysSteerBackFlt;    //后转向系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysMotorLfFlt;  //左前电机系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysMotorRfFlt;  //右前电机系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysMotorLrFlt;  //左后电机系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysMotorRrFlt;  //右后电机系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysBmsFlt;  //BMS系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
    uint8_t VCU_SysDcFlt;   //DC系统故障 "0:normal / 1:fault level 1 / 2:fault level 2 / 3:fault level 3 / 4:fault level 4"
};

//车辆车身及灯光反馈
struct MSG_ID536 {
    uint8_t VCU_VehicleLeftLampFb;  //左转向灯状态反馈 "0:off / 1:on"
    uint8_t VCU_VehicleRightLampFb; //右转向灯状态反馈 "0:off / 1:on"
    uint8_t VCU_VehicleHazardWarLampFb; //危险警示灯开关状态 "0:off / 1:on"
};

//车辆工作状态反馈
struct MSG_ID534 {
    uint8_t VCU_DrivingModeFb;  //驾驶模式反馈 "0:standby / 1:self driving / 2:remote / 3:man"
    uint8_t VCU_ChassisPowerStaFb;  //车辆上电状态反馈 "0:init / 1:on acc / 2:ready / 3:off"
    uint8_t VCU_ChassisPowerDcSta;  //DC工作状态 "0:off / 1:on / 2:standby"
    float VCU_ChassisLowPowerVoltSta; //低压蓄电池电压
    uint8_t VCU_ChassisEStopStaFb;  //紧急停车状态反馈 "0:no / 1:chassis estop / 2:remote estop / 3:chassis err estop"
    uint8_t VCU_CrashFrontSta;  //车辆前碰撞传感器反馈 "0:off / 1:collide"
    uint8_t VCU_CrashRearSta;   //车辆后碰撞传感器反馈 "0:off / 1:collide"
    uint8_t VCU_CrashLeftSta;   //车辆左碰撞传感器反馈 "0:off / 1:collide"
    uint8_t VCU_CrashRightSta;  //车辆右碰撞传感器反馈 "0:off / 1:collide"
    uint8_t VCU_Life;   //VCU循环计数
    uint8_t VCU_CheckSum;   //校验sum=byte0 xor byte1 xor...byte6

};



void frame_parsing_ID530(const unsigned char* frame_data, struct MSG_ID530& msg);
void frame_parsing_ID531(const unsigned char* frame_data, struct MSG_ID531& msg);
void frame_parsing_ID532(const unsigned char* frame_data, struct MSG_ID532& msg);
void frame_parsing_ID534(const unsigned char* frame_data, struct MSG_ID534& msg);
void frame_parsing_ID535(const unsigned char* frame_data, struct MSG_ID535& msg);
void frame_parsing_ID536(const unsigned char* frame_data, struct MSG_ID536& msg);
void frame_parsing_ID537(const unsigned char* frame_data, struct MSG_ID537& msg);
void frame_parsing_ID539(const unsigned char* frame_data, struct MSG_ID539& msg);

bool frame_encapsulation_ID130(const struct MSG_ID130& msg, struct can_frame& frame);
bool frame_encapsulation_ID131(const struct MSG_ID131& msg, struct can_frame& frame);
bool frame_encapsulation_ID132(const struct MSG_ID132& msg, struct can_frame& frame);
bool frame_encapsulation_ID133(const struct MSG_ID133& msg, struct can_frame& frame);


#endif // CAN_PROTOCOL_H

