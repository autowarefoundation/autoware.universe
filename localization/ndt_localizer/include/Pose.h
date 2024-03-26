#pragma once
#include "Point.h"
#include "Quaternion.h"
namespace geometry_msgs
{
    struct Pose
    {
        Point position;
        Quaternion orientation;
    };

}