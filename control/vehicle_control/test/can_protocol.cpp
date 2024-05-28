
#include "can_protocol.h"
#include <iostream>
//#include "v_ros_msg_interface.h"
using namespace std;

/********************************************控制报文封装函数 begin**********************************************/

//驱动控制报文封装
bool frame_encapsulation_ID130(const struct MSG_ID130& msg, struct can_frame& frame){
    memset(&frame,0,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x130;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数

    static int count = 0;
    frame.data[0] = static_cast<uint8_t>(msg.ACU_ChassisDriverEnCtrl & 0x01) + static_cast<uint8_t>((msg.ACU_ChassisDriverModeCtrl & 0x03) << 2) + static_cast<uint8_t>((msg.ACU_ChassisGearCtrl & 0x0f) << 4);
    frame.data[1] = static_cast<uint8_t>((int)(msg.ACU_ChassisSpeedCtrl * 100) & 0x00ff);
    frame.data[2] = static_cast<uint8_t>(((int)(msg.ACU_ChassisSpeedCtrl * 100) & 0xff00) >> 8);
    frame.data[3] = static_cast<uint8_t>((int)(msg.ACU_ChassisThrottlePdlTarget * 10) & 0x00ff);
    frame.data[4] = static_cast<uint8_t>(((int)(msg.ACU_ChassisThrottlePdlTarget * 10)& 0x0300) >> 8);
    frame.data[5] = 0x00;
    frame.data[6] = count;
    count = (++count > 0x0F) ? (count = 0) : count;
    frame.data[7] = (frame.data[0] ^ frame.data[1] ^ frame.data[2] ^ frame.data[3] ^ frame.data[4] ^ frame.data[5] ^ frame.data[6]);
    return true;
}

//制动控制报文封装
bool frame_encapsulation_ID131(const struct MSG_ID131& msg, struct can_frame& frame){
    memset(&frame,0x00,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x131;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数

    static int count = 0;
    frame.data[0] = static_cast<uint8_t>(msg.ACU_ChassisBrakeEn & 0x01) + static_cast<uint8_t>((msg.ACU_ChassisAebCtrl & 0x01) << 4);
    frame.data[1] = static_cast<uint8_t>((int)(msg.ACU_ChassisBrakePdlTarget * 10) & 0x00ff);
    frame.data[2] = static_cast<uint8_t>(((int)(msg.ACU_ChassisBrakePdlTarget * 10) & 0x0300) >> 8);
    frame.data[3] = static_cast<uint8_t>((int)msg.ACU_ChassisEpbCtrl & 0x03);
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = count;
    count = (++count > 0x0F) ? (count = 0) : count;
    frame.data[7] = (frame.data[0] ^ frame.data[1] ^ frame.data[2] ^ frame.data[3] ^ frame.data[4] ^ frame.data[5] ^ frame.data[6]);
    return true;
}

//转向控制报文封装
bool frame_encapsulation_ID132(const struct MSG_ID132& msg, struct can_frame& frame){
    memset(&frame,0x00,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x132;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数

    static int count = 0;
    frame.data[0] = static_cast<uint8_t>(msg.ACU_ChassisSteerEnCtrl & 0x01) + static_cast<uint8_t>((msg.ACU_ChassisSteerModeCtrl & 0x0f) << 4);
    frame.data[1] = static_cast<uint8_t>((int)(msg.ACU_ChassisSteerAngleTarget) & 0x00ff);
    frame.data[2] = static_cast<uint8_t>(((int)(msg.ACU_ChassisSteerAngleTarget) & 0xff00) >> 8);
    frame.data[3] = static_cast<uint8_t>((int)(msg.ACU_ChassisSteerAngleRearTarget) & 0x00ff);
    frame.data[4] = static_cast<uint8_t>(((int)(msg.ACU_ChassisSteerAngleRearTarget) & 0xff00) >> 8);
    frame.data[5] = static_cast<uint8_t>(((msg.ACU_ChassisSteerAngleSpeedCtrl > 500 ? 500 : msg.ACU_ChassisSteerAngleSpeedCtrl) / 2) & 0xff);
    frame.data[6] = count;
    count = (++count > 0x0F) ? (count = 0) : count;
    frame.data[7] = (frame.data[0] ^ frame.data[1] ^ frame.data[2] ^ frame.data[3] ^ frame.data[4] ^ frame.data[5] ^ frame.data[6]);
    return true;
}

//车身控制1
bool frame_encapsulation_ID133(const struct MSG_ID133& msg, struct can_frame& frame){
    memset(&frame,0x00,sizeof (frame));                             //清空消息缓存区
    frame.can_id    =   0x133;                                      //设置消息ID
    frame.can_dlc   =   8;                                          //设置数据字节数
    
    frame.data[0] = static_cast<uint8_t>(msg.ACU_VehicleLeftLampCtrl << 2);
    frame.data[0] += static_cast<uint8_t>(msg.ACU_VehicleRightLampCtrl << 3);
    return true;
}

/********************************************状态报文解析函数 begin**********************************************/

//车辆驱动状态反馈解析
void frame_parsing_ID530(const unsigned char *frame_data, struct MSG_ID530 &msg){
    msg.VCU_ChassisDriverEnSta = (frame_data[0] & 0x01);
    msg.VCU_ChassisDiverSlopover = (frame_data[0] & 0x02) >> 1;
    msg.VCU_ChassisDriverModeSta = (frame_data[0] & 0x0C) >> 2;
    msg.VCU_ChassisGearFb = (frame_data[0] & 0x30) >> 4;
    msg.VCU_ChassisSpeedFb = (float)((frame_data[2] << 8) + frame_data[1]) * 0.01f;
    msg.VCU_ChassisThrottlePaldFb = (float)(((frame_data[4] & 0x03) << 8) + (frame_data[3])) * 0.1f;
    msg.VCU_ChassisAccelerationFb = (float)((frame_data[6] << 8) + (frame_data[5])) * 0.01f;
    // std::cout<<"VCU_ChassisDriverEnSta:"<<(float)msg.VCU_ChassisDriverEnSta<<std::endl;
    // std::cout<<"VCU_ChassisDiverSlopover:"<<(float)msg.VCU_ChassisDiverSlopover<<std::endl;
    // std::cout<<"VCU_ChassisDriverModeSta:"<<(float)msg.VCU_ChassisDriverModeSta<<std::endl;
    // std::cout<<"VCU_ChassisGearFb:"<<(float)msg.VCU_ChassisGearFb<<std::endl;
    // std::cout<<"VCU_ChassisSpeedFb:"<<(float)msg.VCU_ChassisSpeedFb<<std::endl;
    // std::cout<<"VCU_ChassisThrottlePaldFb:"<<(float)msg.VCU_ChassisThrottlePaldFb<<std::endl;
    // std::cout<<"VCU_ChassisAccelerationFb:"<<(float)msg.VCU_ChassisAccelerationFb<<std::endl;

    
}

//车辆制动状态反馈解析
void frame_parsing_ID531(const unsigned char *frame_data, struct MSG_ID531 &msg){
    msg.VCU_ChassisBrakeEnSta = (frame_data[0] & 0x01);
    msg.VCU_VehicleBrakeLampFb = (frame_data[0] & 0x04) >> 2;
    msg.VCU_ChassisEpbFb = (frame_data[0] & 0x30) >> 4;
    msg.VCU_ChassisBrakePadlFb = (((frame_data[2] & 0x03) << 8) + (frame_data[1])) * 0.1f;
    msg.VCU_AebEnStaFb = (frame_data[3] & 0x01);
    msg.VCU_AebTriggerStaFb = (frame_data[3] & 0x04) >> 2;
    // std::cout<<"VCU_ChassisBrakeEnSta:"<<(float)msg.VCU_ChassisBrakeEnSta<<std::endl;
    // std::cout<<"VCU_VehicleBrakeLampFb:"<<(float)msg.VCU_VehicleBrakeLampFb<<std::endl;
    // std::cout<<"VCU_ChassisEpbFb:"<<(float)msg.VCU_ChassisEpbFb<<std::endl;
    // std::cout<<"VCU_ChassisBrakePadlFb:"<<(float)msg.VCU_ChassisBrakePadlFb<<std::endl;
    // std::cout<<"VCU_AebEnStaFb:"<<(float)msg.VCU_AebEnStaFb<<std::endl;
    // std::cout<<"VCU_AebTriggerStaFb:"<<(float)msg.VCU_AebTriggerStaFb<<std::endl;

}

//车辆转向状态反馈解析
void frame_parsing_ID532(const unsigned char *frame_data, struct MSG_ID532 &msg){
    msg.VCU_ChassisSteerEnSta = (frame_data[0] & 0x01);
    msg.VCU_ChassisSteerSlopover = (frame_data[0] & 0x02) >> 1;
    msg.VCU_ChassisSteerWorkMode = (frame_data[0] & 0x0C) >> 2;
    msg.VCU_ChassisSteerModeFb = (frame_data[0] & 0xf0) >> 4;
    msg.VCU_ChassisSteerAngleFb = static_cast<int16_t>(frame_data[2] << 8) + static_cast<int16_t>(frame_data[1]);
    msg.VCU_ChassisSteerAngleRearFb = static_cast<int16_t>(frame_data[4] << 8) + static_cast<int16_t>(frame_data[3]);
}

//车辆工作状态反馈解析
void frame_parsing_ID534(const unsigned char *frame_data, struct MSG_ID534 &msg){
    msg.VCU_DrivingModeFb = (frame_data[0] & 0x03);
    msg.VCU_CrashFrontSta = (frame_data[5]&0x0f)>>3;
    msg.VCU_CrashRearSta = (frame_data[5]&0x07)>>2;
    msg.VCU_CrashLeftSta = (frame_data[5]&0x03)>>1;
    msg.VCU_CrashRightSta = (frame_data[5]&0x01);
    //略
    // std::cout<<"VCU_DrivingModeFb:"<<(float)msg.VCU_DrivingModeFb<<std::endl;
    // std::cout<<"VCU_CrashFrontSta:"<<(float)msg.VCU_CrashFrontSta<<std::endl;
    // std::cout<<"VCU_CrashRearSta:"<<(float)msg.VCU_CrashRearSta<<std::endl;
    // std::cout<<"VCU_CrashLeftSta:"<<(float)msg.VCU_CrashLeftSta<<std::endl;
    // std::cout<<"VCU_CrashRightSta:"<<(float)msg.VCU_CrashRightSta<<std::endl;

    
}

//车辆动力电源状态反馈解析
void frame_parsing_ID535(const unsigned char *frame_data, struct MSG_ID535 &msg){
    //略
}

//车辆车身及灯光反馈解析
void frame_parsing_ID536(const unsigned char *frame_data, struct MSG_ID536 &msg){
    //略
}

//车辆警告和故障解析
void frame_parsing_ID537(const unsigned char *frame_data, struct MSG_ID537 &msg){
    //略
}

//车轮转速反馈解析
void frame_parsing_ID539(const unsigned char *frame_data, struct MSG_ID539 &msg){
    //略
}

