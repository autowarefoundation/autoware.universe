#ifndef LANELINE_H
#define LANELINE_H

#include <iostream>
#include <cstdint>
#include <vector>
using namespace std;

struct Point_h
{
    double x;
    double y;
    double z;
};

struct LaneLine_h
{
    uint32_t id;
    uint8_t typy;
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<Point_h> points;
    uint8_t current_lane_num;
};

#endif
