#ifndef VEHICLESTAT_H
#define VEHICLESTAT_H

#include <iostream>
#include <cstdint>

struct VehicleStat_h
{
    float   VehicleSpeed;               //车速，单位km/h
    int8_t GearShiftPositon;                //换挡器位置
    uint8_t ParkSts;                        //驻车状态
    // ErrorCode 各位为1代表含义如下：
    //     0：不能收到can报文
    //     1：VcuError不能进入自动驾驶状态
    //     2：横向eps故障
    //     3：纵向ebs故障
    //     4：
    //     5：
    //     6：
    //     7：
    uint8_t ErrorCode;                      //故障码
    uint16_t RemainingMile;                 //剩余里程计
    float TolMileage;                   //累计里程计

    float veh_speed;                       //速度，单位km/h
    float SAS_SteeringAngle;               //方向盘转角速度
    float EMS_EngineSpeed;                 //发动机转速
    float EMS_EngineThrottlePosition;      //发动机节气门位置
    float EMS_AccPedal;                    //加速踏板位置
    uint8_t   EMS_BrakePedalStatus;            //制动踏板状态
    uint8_t   TCU_GearShiftPositon;            //换挡器位置
    uint8_t   VehicleModle;                    //车辆状态：自动驾驶or手动驾驶
};

#endif
