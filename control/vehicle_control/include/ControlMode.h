#ifndef CONTROLMODE_H
#define CONTROLMODE_H

#include <iostream>
#include <cstdint>

struct ControlMode_h
{
    uint8_t      start;                     //qidong
    uint8_t      mode_type;                 //模式类型  00：无， 自动模式：（11：自动生成地图，12：已有地图） 遥控模式：20
    uint8_t      enable;                    //紧急使能  
    uint8_t      brake_enable;              //制动使能  0：制动，1：可行驶
    float    speed_require;             //速度要求
    float    angle_require;             //角度要求
};


#endif
