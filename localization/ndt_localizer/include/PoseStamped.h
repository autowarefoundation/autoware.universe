#pragma once
#include "DiagnosticArray.h"
#include "Pose.h"
namespace geometry_msgs
{
    struct PoseStamped
    {
        Header header;
        Pose pose;
    };

}