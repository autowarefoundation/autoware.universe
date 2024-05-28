//********************************************************************************************
// 作   者-->:  杨 东
// 创建时间-->:  2019.08.05
// 修改时间-->:  2019.08.05
// 版   权-->:  重庆邮电大学\自动化学院\汽车电子工程中心\智能汽车技术研究所
//--------------------------------------------------------------------------------------------
// 文件说明 ：
//    1.修改CAN自动设置启动脚本：sudo gedit /etc/rc.local
//      #inmod
//           sudo modprobe can          #Insert CAN BUS subsystem support module.
//           sudo modprobe can_raw      #Insert Raw CAN protocol module (CAN-ID filtering)
//           sudo modprobe mttcan       #Real CAN interface support
//           sudo modprobe can_dev
//      #set can interface
//           sudo ip link set can0 type can bitrate 500000     #设置波特率500k
//           sudo ip link set can1 type can bitrate 500000     #设置波特率500k
//           sudo ip link set up can1                          #开启can0
//           sudo ip link set up can0                          #开启can0
//     2.常用查看命令
//          ip -details link show can0   ifconfig can0 up  ifconfig can0 down
//          ip -details -statistics link show can0（在设备工作中）
//     3.物理连接引脚说明
//          TX2底盘26号GPIO引脚定义：5 == CAN0 RX 7 == CAN0 TX 15 == CAN1 RX 17 == CAN1 TX
//          云智盒引脚：can0H == 1 can0L == 2  CANGUN == 3   can1H == 4 can1L == 5
//     4.内核模块说明
//          can, can_bcm, can_gw, can_raw   : .ko is under ../kernel/drivers/net/can
//          can_dev                         : .ko is under ../kernel/net/can
//          mttcan                          : .ko is under ../t18x/drivers/staging/mttcan
//**********************************************************************************************


#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <cstring>
#include <cstdint>

//回环与是否接收自己报文设置枚举
enum LOOPBACK_RECV_OWN{
  LOOPBACK_RECV_OWN_OFF_OFF = 0x0,    /*关闭回环、关闭接收自己报文*/
  LOOPBACK_RECV_OWN_ON_OFF  = 0x1,    /*开启回环、关闭接收自己报文*/
  LOOPBACK_RECV_OWN_ON_ON   = 0x3     /*开启回环、开启接收自己报文*/
};


//-----------------------------------------------------------------------
//----------------------------接口声明------------------------------------
//-----------------------------------------------------------------------
/**
 * @brief socketcan_init      初始化socketcan 主要完成创建、绑定、设置回环
 * @param can_dev             CAN设备名称 "can0" "can1"
 * @param flag                flag enum LOOPBACK_RECV_OWN
 * @return                    成功返回 文件描述符fd 失败返回 -1
 */
extern int socketcan_init(const char * can_dev,enum LOOPBACK_RECV_OWN flag);

/**
 * @brief close_socket_can     关闭socketcan设备
 * @param sockcan_fd           CAN的文件描述符 fd
 * @return                     成功返回1 失败返回 -1
 */
extern int sockcan_close(const int32_t fd);

/**
 * @brief socketcan_filter_set 设置需要接收报文滤波
 * @param fd                   CAN的文件描述符 fd
 * @param filter_arry          滤波报文ID数组
 * @param num                  滤波报文数
 * @return                     成功返回1 失败返回 -1
 */
extern int socketcan_filter_set(const int32_t fd,const canid_t * filter_arry,const uint32_t num);

/**
 * @brief read_socketcan_frame 接收CAN报文
 * @param fd                   CAN的文件描述符 fd
 * @param frame                接收缓存区数组
 * @param num                  单次调用接收报文数
 * @return                     成功返回1 失败返回 -1
 */
extern int read_socketcan_frame(const int32_t fd,struct can_frame * frame, const uint32_t num);

/**
 * @brief write_socketcan_frame 发送CAN报文
 * @param fd                    CAN的文件描述符 fd
 * @param frame                 单次调用发送帧结构
 * @return                      成功返回1 失败返回 -1
 */
int write_socketcan_frame(const int32_t fd,const struct can_frame &frame);

#endif // SOCKETCAN_H

