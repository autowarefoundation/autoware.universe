#ifndef TASK_H
#define TASK_H

#include <cstdint>

struct Task_h
{
    uint8_t Is_Avoid_obstacle = 1;
    uint8_t Is_Pass_traffic_light = 2;

    uint8_t task_type;
    float s_start;
    float s_end;
    float info;
    float info_2;
};

#endif

