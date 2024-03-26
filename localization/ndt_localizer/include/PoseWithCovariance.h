#pragma once
#include "Pose.h"
struct PoseWithCovariance
{
    geometry_msgs::Pose pose;
    float covariance[36];
};
