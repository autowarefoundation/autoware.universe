#include "lon_controller.h"

//-->>全局变量
struct VehicleStat veh_stat      = {0,0,0,0,0,0,0};
struct Request     request_value = {LON_STATUS::FORWARD_ENABLE,0,0,0};

//-->>静态全局变量
static struct RtkImuStat rtk_stat = {0,0,0,0};

void set_vehicle_stat(struct VehicleStat &arg){
    veh_stat = arg;
    return ;
}
void set_rtk_imu_stat(struct RtkImuStat &arg){
    rtk_stat = arg;
    return ;
}
void set_requre(struct Request &arg){
    request_value = arg;
    return ;
}

//--------------------------------------------------------
/**
 * @brief stop_solve
 * @return
 */
struct Trq_Bre_Cmd stop_solve(){
  struct Trq_Bre_Cmd temp = {0,0,0,0};
  temp.trq_enable = 0;
  temp.trq_value  = 0;
  temp.bre_enable = 1;
  temp.bre_value  = 100;
  return temp;
}

/**
 * @brief aeb_solve
 * @return
 */
struct Trq_Bre_Cmd aeb_solve() {
    struct Trq_Bre_Cmd temp = {0,0,0,0};
    temp.trq_enable = 0;
    temp.trq_value  = 0;
    temp.bre_enable = 1;
    temp.bre_value  = 300;
    return temp;
}

/**
 * @brief run_PID_solve
 * @return
 */
struct Trq_Bre_Cmd run_solve() {
  struct Trq_Bre_Cmd temp = {0,0,0,0};
  temp.trq_enable = 1;
  temp.trq_value  = request_value.run_speed;
  temp.bre_enable = 0;
  temp.bre_value  = 0;
  return temp;
}

struct Trq_Bre_Cmd back_solve() {
  struct Trq_Bre_Cmd temp = {0,0,0,0};
  temp.trq_enable = 1;
  temp.trq_value  = request_value.run_speed;
  temp.bre_enable = 0;
  temp.bre_value  = 0;
  return temp;
}

