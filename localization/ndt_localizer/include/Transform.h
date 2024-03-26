#pragma once
#include "Vector3.h"
#include "Quaternion.h"

struct Transform
{
    Vector3 translation;
    Quaternion rotation;
};
