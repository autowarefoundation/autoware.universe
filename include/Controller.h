#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <iostream>
#include <cstdint>

struct TrqBreCmd_h
{
    uint8_t trq_enable;        //发动机驱动扭矩是否使能
    uint32_t trq_value_2;       //->>扭矩:所需扭矩值     #2号车使用
    float trq_value_3;         //->>扭矩:所需扭矩百分比  #3号车使用
    uint8_t bre_enable;         //刹车是否使能
    float bre_value;            //刹车值
    uint8_t ACC_DecToStop;      //ACC请求减速到停止 0x0: no demand; 0x1: demand 车子按自己的减速度平滑停车 不受发送的其他指令控制
    uint8_t ACC_Driveoff;       //释放汽缸压力，必须与ACC_DecToStop配合使
};

struct SteeringCmd_h
{
    float SteeringAngle;         //->>方向： 发送方向角度值
};

struct AEBCmd_h
{
    uint8_t AEB_enable;
    uint8_t ACC_DecToStop;
    float AEB_bre_value;
};

struct PRNDShiftCmd_h
{
    uint8_t APA_TransPRNDShiftEnable;
    uint8_t APA_TransPRNDShiftReqValid;
    uint8_t APA_TransPRNDShiftRequest;
};



#endif