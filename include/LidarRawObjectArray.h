#ifndef LIDARRAWOBJECTARRAY_H
#define LIDARRAWOBJECTARRAY_H

#include <iostream>
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include "LidarRawObject.h"
#include "ObjectArray.h"

using namespace std;


struct LidarRawObjectArray_h
{
    Header_h header;
    vector<LidarRawObject_h> objs;
};

#endif