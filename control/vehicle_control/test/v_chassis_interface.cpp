#include "v_chassis_interface.h"
#include <iostream>
#include <cmath>
#include <complex>
#include <chrono>
#include <ctime>
#include <sys/time.h>
using namespace std;




int brake_flag = 0;
int brake_count = 0;
///-->>全局变量----------------------------------------------------------------------------------------------------
int can_fd = 0; // canbus 文件描述符
///-->>静态全局变量-------------------------------------------------------------------------------------------------
// static bool   self_drive_mode = false;                                       //自动驾驶模式

struct RtkImuState rtk_state = {0, 0, 0, 0}; // rtk状态初始化
struct Trq_Bre_Cmd trq_bre_cmd = {0, 0, 0, 0};
struct Control_Mode control_mode = {0, 0, 0, 0, 0, 0, 0};
uint8_t turn_light = 0;

///--控制消息>>---------------------------------------------------------------------------------------------------
static struct MSG_ID131 mgs_id131 = {0, 0, 0, 0};    // 基础模式⾃动驾驶下 发控制指令
static struct MSG_ID130 mgs_id130 = {0, 0, 0, 0, 0}; // 高级模式自动驾驶 发控制指令
static struct MSG_ID132 mgs_id132 = {0, 0, 0, 0, 0}; // 基础/⾼级模式 转向控制状态指令
static struct MSG_ID133 mgs_id133 = {0, 0, 0};       // ⾼级模式⾃动驾驶下 发动⼒控制指令

///--报文解析>>---------------------------------------------------------------------------------------------------
static struct MSG_ID531 mgs_id531 = {0, 0, 0, 0, 0, 0};                // 基础车辆回传数据
static struct MSG_ID539 mgs_id539 = {0, 0, 0, 0};                      // 基础⻋辆状态参数回传（暂未开放）
static struct MSG_ID530 mgs_id530 = {0, 0, 0, 0, 0, 0, 0};             // ⾼级模式⻋辆转向状态反馈（暂未开放）
static struct MSG_ID535 mgs_id535 = {0, 0, 0, 0, 0, 0};                // ⾼级模式⻋辆动⼒状态轮速反馈（暂未开放）
static struct MSG_ID532 mgs_id532 = {0, 0, 0, 0, 0, 0};                // ⾼级模式⻋辆动⼒状态反馈（暂未开放）
static struct MSG_ID537 mgs_id537 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // ⾼级模式⻋辆制动状态反馈（暂未开放）
static struct MSG_ID536 mgs_id536 = {0, 0, 0};
static struct MSG_ID534 mgs_id534 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // ⾼级模式⻋辆制动状态反馈（暂未开放）

void vehicle_ready()
{
    mgs_id130.ACU_ChassisDriverEnCtrl = 1;   // 加速控制使能
    mgs_id130.ACU_ChassisDriverModeCtrl = 0; // 速度控制模式
    mgs_id132.ACU_ChassisSteerModeCtrl = 0;  // front ackerman
    mgs_id132.ACU_ChassisSteerEnCtrl = 1;    // 车辆转向控制使能
}

void set_rtk_imu_state(struct RtkImuState &arg)
{
    rtk_state = arg;
    return;
}

void set_control_mode(struct Control_Mode &arg)
{
    control_mode = arg;
    return;
}

// 获取方向角度值
void set_steering_cmd(const float angle)
{
    int16_t temp = 0;
    // 对角度进行范围检查以后，赋值处理 【-1000，,1000】
    if ((control_mode.mode_type) == 0x20)
    { // 遥控
        // mgs_id132.ACU_ChassisSteerModeCtrl = 2;
        temp = static_cast<int16_t>(control_mode.angle_require * 10.4f);
        std::cout << "control_mode.angle_require = " << control_mode.angle_require << std::endl;
        std::cout << "temp =  " << temp << std::endl;
        mgs_id132.ACU_ChassisSteerAngleTarget = fabs(temp) <= 500.0f ? temp : (fabs(temp) / temp) * 500.0f;
        mgs_id132.ACU_ChassisSteerAngleRearTarget = -mgs_id132.ACU_ChassisSteerAngleTarget;
    }
    else
    {
        mgs_id132.ACU_ChassisSteerAngleTarget = fabs(angle) <= 500.0f ? angle : (fabs(angle) / angle) * 500.0f;
    }
    return;
}
///-------------------------------------------------------------------------------------------------------------
uint8_t brakeClear = 0;
void set_trq_bre_cmd(struct Trq_Bre_Cmd &msg)
{
    // 遥控 or 跟随
    if ((control_mode.mode_type) == 0x20)
    { // 遥控
        // ROS_INFO_THROTTLE(1, "========remote======");
        //  //紧急制动
        if (control_mode.enable > 0)
        {
            mgs_id130.ACU_ChassisSpeedCtrl = 0x00;
            mgs_id131.ACU_ChassisBrakeEn = 1;
            mgs_id131.ACU_ChassisBrakePdlTarget = 50;
        }
        else
        { // 正常行驶
            // 刹车
            if (control_mode.brake_enable > 0)
            {
                mgs_id130.ACU_ChassisSpeedCtrl = 0x00;
                mgs_id131.ACU_ChassisBrakeEn = 1;
                mgs_id131.ACU_ChassisBrakePdlTarget = 50;
            }
            else
            { // 不刹车行驶
                // 放手刹
                mgs_id131.ACU_ChassisBrakeEn = 0;
                mgs_id131.ACU_ChassisAebCtrl = 0;
                mgs_id131.ACU_ChassisBrakePdlTarget = 0;
                // ROS_INFO_THROTTLE(1, "mgs_id130.ACU_ChassisSpeedCtrl = %f", mgs_id130.ACU_ChassisSpeedCtrl);
                if (control_mode.speed_require > 0.1)
                {
                    mgs_id130.ACU_ChassisSpeedCtrl = control_mode.speed_require / 3.6; // 0-60 KM/H;
                    mgs_id130.ACU_ChassisGearCtrl = 1;
                }
                else if (control_mode.speed_require < -0.1)
                {
                    mgs_id130.ACU_ChassisSpeedCtrl = -control_mode.speed_require / 3.6; // 0-60 KM/H;
                    mgs_id130.ACU_ChassisGearCtrl = 3;
                }
                else
                {
                    mgs_id130.ACU_ChassisSpeedCtrl = 0 / 3.6; // 0-60 KM/H;
                    mgs_id130.ACU_ChassisGearCtrl = 1;
                }
                mgs_id130.ACU_ChassisGearCtrl = 1;
            }
        }
    }
    else
    { // 0x00
        // ROS_INFO_THROTTLE(1, "========Auto --- flow======");
        trq_bre_cmd = msg;
        if (trq_bre_cmd.bre_enable == 1)
        {
            brake_flag = 1;
            mgs_id130.ACU_ChassisSpeedCtrl = 0x00;
            mgs_id131.ACU_ChassisBrakeEn = 1;
            mgs_id131.ACU_ChassisBrakePdlTarget = 50;
        }
        else
        {
            // 放手刹
            if (brake_flag == 1)
            {
                brakeClear = 1;
                brake_flag = 0;
            }
            mgs_id131.ACU_ChassisBrakeEn = 0;
            mgs_id131.ACU_ChassisAebCtrl = 0;
            mgs_id131.ACU_ChassisBrakePdlTarget = 0;
            if (trq_bre_cmd.trq_value >= 0)
            {
                // mgs_id183.speed_value = trq_bre_cmd.trq_value;//0-60 KM/H;
                mgs_id130.ACU_ChassisGearCtrl = 1;
                mgs_id130.ACU_ChassisSpeedCtrl = trq_bre_cmd.trq_value / 3.6;
                // ROS_INFO_THROTTLE(1, "mgs_id130.ACU_ChassisSpeedCtrl = %f", mgs_id130.ACU_ChassisSpeedCtrl);
            }
            else
            {
                // mgs_id183.speed_value = -trq_bre_cmd.trq_value;//0-60 KM/H;
                mgs_id130.ACU_ChassisGearCtrl = 3;
                mgs_id130.ACU_ChassisSpeedCtrl = trq_bre_cmd.trq_value / 3.6;
            }
        }
    }
}

static void *status_monitor_thread_recall(void *arg);

/**
 * @brief manual_auto_driving_thread_recall
 * @param arg
 * @return
 */

static void msg_send_recall(int signo);

//-->>以下方法为软件框架，不需要优化
/**
 * @brief start_status_monitor_thread 开启车辆状态监控线程
 * @return
 */
pthread_t start_status_monitor_thread()
{
    pthread_t id = 0;
    // 开启获取车辆状态线程
    if (pthread_create(&id, nullptr, status_monitor_thread_recall, nullptr) != 0)
    {
        std::cerr << "create getCarInforThread thread fail!" << std::endl;
        exit(-1);
    }
    return id;
}

/**
 * @brief start_send_timer开启控制报文发送定时器
 * @param t_ms
 */
void start_send_timer(int t_ms)
{
    signal(SIGALRM, msg_send_recall); // SIGALRM是信号类型，收到就执行后面的函数
    struct itimerval it, oldit;
    it.it_value.tv_sec = 0;               // 第一次执行时间初始时间
    it.it_value.tv_usec = t_ms * 1000;    // 20ms
    it.it_interval.tv_sec = 0;            // 间隔时间
    it.it_interval.tv_usec = t_ms * 1000; // 20ms

    if (-1 == setitimer(ITIMER_REAL, &it, &oldit)) //
    {
        perror("msg_send_timer_init failed");
        exit(-1);
    }
    return;
}

/**
 * @brief msg_send_recall
 * @param signo
 */
static void msg_send_recall(int signo)
{
    if (signo != SIGALRM)
        return;
    vehicle_ready();
    static struct can_frame frame; // 发送帧结构

    //--------------------------------------------------------------------------------
    // //-->>发送控制消息
    if (
        (brakeClear == 1) ||
        (mgs_id531.VCU_ChassisBrakePadlFb > 0 && mgs_id130.ACU_ChassisSpeedCtrl != 0))
    {
        // std::cout<<"-----------------"<<std::endl;
        mgs_id131.ACU_ChassisBrakeEn = 1;
        mgs_id131.ACU_ChassisBrakePdlTarget = 0;
        mgs_id130.ACU_ChassisSpeedCtrl = 0;
        brakeClear = 0;
    }
    memset(&frame, 0, sizeof(frame)); 
    mgs_id130.ACU_ChassisDriverEnCtrl = 1;
    mgs_id130.ACU_ChassisSpeedCtrl = 1;           // 清空缓存区
    frame_encapsulation_ID130(mgs_id130, frame); // 填充帧消息
//-->>条件编译
#ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd, frame); // 发送帧消息
#endif

    memset(&frame, 0, sizeof(frame)); // 清空缓存区
    // 刹车标志置零前需保持刹车标志不变再将刹车值置零

    frame_encapsulation_ID131(mgs_id131, frame); // 填充帧消息
//-->>条件编译
#ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd, frame); // 发送帧消息
#endif

    memset(&frame, 0, sizeof(frame));
    // mgs_id132.ACU_ChassisSteerEnCtrl = 1;
    // mgs_id132.ACU_ChassisSteerAngleTarget = 500;  // 清空缓存区
    frame_encapsulation_ID132(mgs_id132, frame); // 填充帧消息
    // auto start = std::chrono::system_clock::now(); // 获取当前时间
    // std::time_t pub_angle_time = std::chrono::system_clock::to_time_t(start);
    // std::cout << "The pub_angle_time time is: " << std::ctime(&pub_angle_time);
    // struct timeval tv;
    // gettimeofday(&tv, NULL);
    // cout << "The pub_angle_timee mil time is: " << tv.tv_sec*1000 + tv.tv_usec/1000 << endl;
                

//-->>条件编译
#ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd, frame); // 发送帧消息
#endif

    // 灯光控制报文封装发送
    memset(&frame, 0, sizeof(frame)); // 清空缓存区
    // mgs_id133.ACU_VehicleLeftLampCtrl = 0;
    // mgs_id133.ACU_VehicleRightLampCtrl = 1;
    frame_encapsulation_ID133(mgs_id133, frame); // 填充帧消息
//-->>条件编译
#ifdef NO_LOCAL_DEBUG
    write_socketcan_frame(can_fd, frame); // 发送帧消息
#endif

    // ros::spinOnce();
    return;
}

/**
 * @brief set_TurnLight_cmd
 * @param TurnLight
 */
void set_TurnLight_cmd(const uint8_t TurnLight)
{
    turn_light = TurnLight;
    switch (turn_light)
    {
    case 0:
        mgs_id133.ACU_VehicleLeftLampCtrl = 0;
        mgs_id133.ACU_VehicleRightLampCtrl = 0;
        break;
    case 1:
        mgs_id133.ACU_VehicleLeftLampCtrl = 1;
        mgs_id133.ACU_VehicleRightLampCtrl = 0;
        break;
    case 2:
        mgs_id133.ACU_VehicleLeftLampCtrl = 0;
        mgs_id133.ACU_VehicleRightLampCtrl = 1;
        break;
    case 3:
        mgs_id133.ACU_VehicleLeftLampCtrl = 1;
        mgs_id133.ACU_VehicleRightLampCtrl = 1;
        break;
    }
    return;
}

/**
 * @brief get_veh_status
 * @param stutus
 */
void get_veh_status(struct VehicleStat &stutus)
{
    stutus.VehicleSpeed = rtk_state.speed2d;            // mgs_id193.speed_feedback;
    stutus.VehicleSpeed = mgs_id530.VCU_ChassisSpeedFb; // mgs_id193.speed_feedback;

    // ROS_INFO_THROTTLE(2, "VehicleSpeed= %f", stutus.VehicleSpeed);
    return;
}

/**
 * @brief status_monitor_thread_recall
 * @param arg
 * @return
 */
void *status_monitor_thread_recall(void *arg)
{
    struct can_frame frame;
    while (true)
    {
        memset(&frame, 0, sizeof(struct can_frame)); // 清空接收缓存区
//-->>条件编译
#ifdef NO_LOCAL_DEBUG
        read(can_fd, &frame, sizeof(frame));
#endif
        switch (frame.can_id)
        {
        case 0x530:
        {
            frame_parsing_ID530(frame.data, mgs_id530); // 解析
            break;
        }
        case 0x531:
        {
            frame_parsing_ID531(frame.data, mgs_id531);
            break;
        }
        case 0x532:
        {
            frame_parsing_ID532(frame.data, mgs_id532); // 解析
            // ROS_INFO_THROTTLE(1, "VCU_ChassisSteerAngleFb= %d", mgs_id532.VCU_ChassisSteerAngleFb);
            // ROS_INFO_THROTTLE(1, "VCU_ChassisSteerAngleRearFb= %d", mgs_id532.VCU_ChassisSteerAngleRearFb);
            break;
        }
        case 0x534:
        {
            frame_parsing_ID534(frame.data, mgs_id534);
            break;
        }
        default:
            break;
        }
    }
    return arg;
}
