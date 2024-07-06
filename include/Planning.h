#ifndef PLANNING_H
#define PLANNING_H

#include <iostream>
#include <cstdint>
#include<vector>

using namespace std;

struct Request_h
{
    uint8_t reques_type;      //前进使能
    float run_speed;        //行车速度
    float stop_distance;    //前方停止距离
    float aeb_distance;     //前方AEB停止距离

    uint8_t FORWARD_ENABLE_H = 0;     //前进使能
    uint8_t BACK_ENABLE_H = 1;    //倒车使能
    uint8_t STOP_ENABLE_H = 2;   //停车使能
    uint8_t AEB_ENABLE_H = 3;     //紧急刹车使能
};

struct Path_h
{
    vector<float> x_ref;
    vector<float> y_ref;
};






#endif
