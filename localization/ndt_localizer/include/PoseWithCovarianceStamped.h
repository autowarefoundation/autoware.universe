#pragma once
#include "DiagnosticArray.h"
#include "PoseWithCovariance.h"
namespace geometry_msgs
{
    struct PoseWithCovarianceStamped
    {
        Header header;
        PoseWithCovariance pose;
    };
};
