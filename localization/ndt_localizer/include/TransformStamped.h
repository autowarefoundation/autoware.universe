#pragma once
#include "Transform.h"
#include "DiagnosticArray.h"
namespace geometry_msgs
{
    struct TransformStamped
    {
        Header header;
        string child_frame_id;
        Transform transform;
    };
};
