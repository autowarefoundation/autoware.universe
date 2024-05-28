#include "socketcan.h"


//-------------------------------------------------------------------------------------------------------------
/**
 * @brief socketcan_init
 * @param can_dev
 * @param flag
 * @return
 */
int socketcan_init(const char * can_dev,enum LOOPBACK_RECV_OWN flag){
    //创建 SocketCAN 套接字
    int32_t fd = 0;
    if ((fd = socket(PF_CAN,SOCK_RAW,CAN_RAW)) < 0){
        perror("socketcan open failed");
        return -1;
    }
    //指定 can 设备
    struct ifreq sockcan_conf;
    strcpy(sockcan_conf.ifr_name,can_dev);
    ioctl(fd,SIOCGIFINDEX,&sockcan_conf);
    //将套接字地址与 can 绑定
    struct sockaddr_can addr;
    bzero(&addr,sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = sockcan_conf.ifr_ifindex;
    if (bind(fd, (struct sockaddr *) &addr, sizeof(addr))){
       perror("sockaddr_can bind failed");
        return -1;
    }
    // //回环功能设置
    // int loop_back = flag & 0x01;  // 0 表示关闭, 1 表示开启( 默认)
    // if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loop_back, sizeof(loop_back))){
    //     perror("socketcan CAN_RAW_LOOPBACK setting failed");
    //     return -1;
    // }
    // //回环开启情况下，设置是否接收自己的报文
    //  int recv_own_msgs = (flag & 0x02)>>1; // 0 表示关闭( 默认), 1 表示开启（回环开启情况下可以选择置1）
    //  if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs))){
    //      perror("socketcan CAN_RAW_RECV_OWN_MSGS setting failed");
    //      return -1;
    //  }
    return fd;
}


//-------------------------------------------------------------------------------------------------------------
/**
 * @brief socketcan_filter_set
 * @param fd
 * @param filter_arry
 * @param num
 * @return
 */
int socketcan_filter_set(const int32_t fd,const canid_t * filter_arry,const uint32_t num){
  //屏蔽所有帧
  if(num == 0){
    if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0)){
      perror("frame filter set err");
      return -1;
    }
  }
  //接收部分帧
  else {
    struct can_filter rfilter[num];
    for(uint32_t i = 0; i< num; i++){
      rfilter[i].can_id =  filter_arry[i];   //符号位 CAN_INV_FILTER 在置位时可以实现 can_id 在执行过滤前的位反转
      rfilter[i].can_mask = CAN_SFF_MASK;
    }
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter))){
      perror("frame filter set err");
      return -1;
    }
  }
  //通过错误掩码可以实现对错误帧的过滤
  can_err_mask_t err_mask = ( CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF );
  if(setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,&err_mask, sizeof(err_mask))){
    perror("frame filter set err");
    return -1;
  }
  return 1;
}

//-------------------------------------------------------------------------------------------------------------

/**
 * @brief sockcan_close
 * @param sockcan_fd
 * @return
 */
int sockcan_close(const int32_t fd){
    if(fd <= 0){
      perror("socketcan fd invail");
      return -1;
    }

    if(-1 == close(fd)){
      perror("socktcan device close failed");
      return -1;
    }
    return 1;
}

//-------------------------------------------------------------------------------------------------------------

/**
 * @brief read_socketcan_frame
 * @param fd
 * @param frame
 * @param num
 * @return
 */
int read_socketcan_frame(const int32_t fd,struct can_frame * frame, const uint32_t num){
    size_t byte_num = sizeof(struct can_frame) *num;  //计算需要接收的字节数
    bzero(frame,byte_num); //清空接收缓存区
    if (byte_num != read(fd,frame,byte_num)){
        perror("frame read err");
        return -1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------

/**
 * @brief write_socketcan_frame
 * @param fd
 * @param frame
 * @return
 */
int write_socketcan_frame(const int32_t fd,const struct can_frame &frame){
    int byte_num = sizeof(struct can_frame);
    if (byte_num != write(fd,&frame,sizeof(struct can_frame))){
      perror("frame send err1116");
      return -1;
    }
    return 0;
}

