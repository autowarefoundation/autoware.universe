#ifndef RCONTROLSRV_H
#define RCONTROLSRV_H

#include <cstdint>
#include <string>
using namespace std;

struct Controlsrv_h
{
    uint8_t Is_stop = 1;
    uint8_t Is_aeb = 2;
    uint8_t Is_back = 3;
    uint8_t Is_swith_speed = 4;


    uint8_t type;
    bool enable;
    float info;
    string source;
};


#endif