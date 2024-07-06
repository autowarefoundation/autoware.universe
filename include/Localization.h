#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <iostream>
#include <cstdint>

struct NaviData_h
{
    float pitch;          //俯仰角
    float roll;           //翻滚角
    float heading;         //航向角

    double longitude;      //经度
    double latitude;      //纬度
    double altitude;      //海拔

    float speed2d;         //车辆速度
// ------------------------------
    float Ve;

    float Vn;

    float Vu;
// --------------------------------
    int32_t  pose_type;     //定位状态
    double gpsTime;       

    int32_t INS_Status;       //解算状态
    float Lat_vari;         //纬度标准差
    float Lon_vari;         //经度标准差
};

struct CurPose_h
{
    double x;
    double y;
    double theta;
    double s;
    double d;
};


#endif