#ifndef OBJECTARRAY_H
#define OBJECTARRAY_H

#include <iostream>
#include <cstdint>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include "Object.h"

using namespace std;

struct Header_h
{
    uint32_t seq;
    double stamp;
    std::string frame_id;
};

struct ObjectArray_h
{
    Header_h header;
    vector<Object_h> objs;
};

#endif