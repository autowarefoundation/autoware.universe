#ifndef LIDARRAWOBJECT_H
#define LIDARRAWOBJECT_H

#include <iostream>
#include <array>
#include <cstdint>
#include <vector>
#include <Eigen/Dense>
#include "LaneLine.h"

using namespace std;

struct LidarRawObject_h
{
    std::array<Point_h, 8> bbox_point;
    Eigen::Vector3f lwh;

    float x_pos;
    float y_pos;
    float z_pos;

    float lidar_x_pos;
    float lidar_y_pos;
    float lidar_z_pos;
};


#endif