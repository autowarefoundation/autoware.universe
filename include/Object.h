#ifndef OBJECT_H
#define OBJECT_H

#include <iostream>
#include <array>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include "LaneLine.h"

using namespace std;

struct Object_h
{
    uint32_t id;
    std::array<Point_h, 8> bbox_point;
    uint8_t type;

    float vx;
    float vy;
    float v;

    Eigen::Vector3f lwh;
    float x_pos;
    float y_pos;

    float s_pos;
    float d_pos;

    float lidar_x_pos;
    float lidar_y_pos;
    float lidar_z_pos;
};


#endif
