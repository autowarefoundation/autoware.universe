extern "C" {
// #include "/home/crp/dora_project/dora-rs/dora/apis/c/operator/operator_api.h"
#include "/home/crp/dora_project/dora-rs/dora/apis/c/operator/operator_api.h"
#include "/home/crp/dora_project/dora-rs/dora/apis/c/operator/operator_types.h"

// #include "../../../dora/apis/c/operator/operator_api.h"
}
/*
cd ~/dora_project/dora-rs/dora-hardware/vendors/lidar/Robosense
clang++ -c rs_driver_dora.cpp -o build/rs_driver_dora.o -fdeclspec -fPIC -I /home/crp/dora_project/dora-rs/dora-hardware/vendors/lidar/Robosense/rs_driver/src/
clang++ -shared build/rs_driver_dora.o -o build/librs_driver_dora.so
clang++ rs_driver_dora.cpp -lm -lrt -ldl -pthread -lpcap -std=c++14 -L ../../target/release --output build/rs_driver_dora -I /home/crp/dora_project/dora-rs/dora-hardware/vendors/lidar/Robosense/rs_driver/src/
*/


#include <memory>
#include <iostream>
#include <vector>
#include <string.h>
#ifdef ENABLE_PCL_POINTCLOUD
#include "./src/rs_driver/msg/pcl_point_cloud_msg.hpp"
#else
#include "rs_driver/src/rs_driver/msg/point_cloud_msg.hpp"
#include "rs_driver/src/rs_driver/api/lidar_driver.hpp"
#endif

/*
If you are trying to read this program, please read the document of lidar drive SDK, it should be at ./doc
Especially 03_thread_model.md is the most important doc, it explains that why it use two queues.
And, this file, is modified from ./demo/demo_online.cpp
I think reading that file will help you a lot.

The only thing I modify in the SDK is 'regExceptionCallback'
I changed its return from void to string. The original function is designed to print error massage to console, so I
changed it to return error massage.

*/

typedef PointXYZI PointT;  // If you want to change the format of point cloud, replace PointXYZI with PointXYZIRT
typedef PointCloudT<PointT> PointCloudMsg;

using namespace robosense::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }
  return std::make_shared<PointCloudMsg>();
}

void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

std::string exceptionCallback(const Error& code)  // 错误报告
{
  return code.toString();
}

Vec_uint8_t processCloud(void)
{
  std::shared_ptr<PointCloudMsg> msg;
  while (1)
  {
    msg = stuffed_cloud_queue.popWait();  // 这个popwait函数是一个线程安全的队列pop
    if (msg.get() == NULL)
    {
      continue;
    }
    else
    {
      break;
    }
  }
  // Well, it is time to process the point cloud msg, even it is time-consuming.
  if (sizeof(PointT) <= 16)
  {
    // 下面这块被注释掉的是非常紧凑、省内存的一种字节输出方式，但是因为访存次数比较多，每0.1秒输出一次，会慢大概0.5毫秒的样子，而且不方便读，所以应该不用了
    // size_t pointSize = 13;
    // size_t cloudSize =
    //     ((msg->points.size()) * pointSize) + 12;  // 4byte for message seq, 8byte for timestamp, others for points

    // u_int8_t *bytePointCloud = new u_int8_t[cloudSize], *pointBegin = bytePointCloud + 12;
    // u_int32_t* seq = (u_int32_t*)bytePointCloud;
    // *seq = msg->seq;
    // double* timestamp = (double*)(bytePointCloud + 4);
    // *timestamp = msg->timestamp;
    // float* coordinates = (float*)(pointBegin);
    // std::vector<PointT>::iterator pointPtr = msg->points.begin();
    // for (int i = 0; i < msg->points.size(); ++i)
    // {
    //   *coordinates = pointPtr[i].x;
    //   *(coordinates + 1) = pointPtr[i].y;
    //   *(coordinates + 2) = pointPtr[i].z;
    //   pointBegin += 13;
    //   *(pointBegin - 1) = pointPtr[i].intensity;
    //   coordinates = (float*)(pointBegin);
    // }

    size_t cloudSize = (((msg->points.size()) + 1) * 16);  // 4byte for message seq, 4bytes empty, 8byte for timestamp,
                                                           // others for points
    u_int8_t* bytePointCloud =
        (u_int8_t*)(new PointT[cloudSize / sizeof(PointT)]);
    u_int32_t* seq = (u_int32_t*)bytePointCloud;
    *seq = msg->seq;
    double* timestamp = (double*)(bytePointCloud + 8);
    *timestamp = msg->timestamp;
    // PointT* point = (PointT*)(bytePointCloud + 16);
    // std::vector<PointT>::iterator pointPtr = msg->points.begin();
    // for (int i = 0; i < msg->points.size(); ++i){
    //   *point++ = pointPtr[i];
    // }
    memcpy(bytePointCloud+16,&(msg->points[0]),cloudSize-16);

    free_cloud_queue.push(msg);
    Vec_uint8_t result;
    result.ptr = bytePointCloud;
    result.len = cloudSize;
    result.cap = cloudSize;
    return result;
  }
  else if (sizeof(PointT) == 24)
  {                                   // just write them here, I didn't test it
    size_t cloudSize =
        ((msg->points.size()) * 24);  // 24 bytes for each point, 4*3 bytes for coordinates, 1 byte for intensity, 1
                                      // byte because of byte aligned 2 bytes for rings, 8 bytes for timestamp

    u_int8_t* bytePointCloud = (u_int8_t*)new PointT[cloudSize / sizeof(PointT)];
    memcpy(bytePointCloud,&(msg->points[0]),cloudSize);
    // PointT* point = (PointT*)(bytePointCloud);
    // std::vector<PointT>::iterator pointPtr = msg->points.begin();
    // for (int i = 0; i < msg->points.size(); ++i)
    // {
    //   *(point++) = pointPtr[i];
    // }

    free_cloud_queue.push(msg);
    Vec_uint8_t result;
    result.ptr = bytePointCloud;
    result.len = cloudSize;
    result.cap = cloudSize;
    return result;
  }
  else
  {
    std::cerr << "point size error! This may happen when your system is not byte aligned!";
    Vec_uint8_t result = { .ptr = NULL };
    result.len = 0;
    result.cap = 0;
    return result;
  }
}

class Operator
{
public:
  Operator();
  ~Operator();
  RSDriverParam param;
  LidarDriver<PointCloudMsg> driver;
};

Operator::Operator()
{
  param.input_type = InputType::ONLINE_LIDAR;  //  input type, I didn't test other type
  param.input_param.msop_port = 6699;          ///< Set the lidar msop port number, the default is 6699
  param.input_param.difop_port = 7788;         ///< Set the lidar difop port number, the default is 7788
  param.lidar_type = LidarType::RSHELIOS_16P;  ///< Set the lidar type. Make sure this type is correct

  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback,
                               driverReturnPointCloudToCallerCallback);  ///< Register the point cloud callback
                                                                         ///< functions
  driver.regExceptionCallback(exceptionCallback);                        ///< Register the exception callback function
}
Operator::~Operator()
{
  driver.stop();
}

extern "C" DoraInitResult_t dora_init_operator()
{
  Operator* op = std::make_unique<Operator>().release();
  DoraInitResult_t result = { .operator_context =
                                  (void*)op }; 
  if (!(op->driver.init(op->param)))  ///< Call the init function
  {
    uint8_t* errorCode = new uint8_t[24];
    errorCode = (uint8_t*)"Driver Initialize Error";
    result.result.error.ptr = errorCode;
    result.result.error.len = 23;
    result.result.error.cap = 24;
  }
  return result;
}

extern "C" DoraResult_t dora_drop_operator(void* operator_context)
{
  delete (Operator*)operator_context; 
  return {}; 
}

extern "C" OnEventResult_t dora_on_event(  
    RawEvent_t* event,  
    const SendOutput_t*
        send_output,  

    void* operator_context)
{
  Operator* op = (Operator*)operator_context;
  // if (event->input != NULL)
  if (1)
  {
    // input event

    op->driver.start();  ///< The driver thread will start
                         // To be honest, I don't know this thread should be put in the dora_init_operator function or
                         // here, but I can explain this function: it collect the packets from lidar until it collect
                         // enough points to make a 360 degree point cloud frame. So I put it here, maybe I'm wrong, but
                         // I am just a math undergraduate who graduated a month ago, my boss gived me two weeks to
                         // learn dora and this lidar, I think I can't solve this problem. I write this paragraph to
                         // complain and try to help you. If you find that the fist first point clouds are shorter than
                         // others, this line of code maybe the reason.

    const char* out_id = "pointCloud";
    char* out_id_heap = strdup(out_id);
    Output_t output;
    output= {.id = {
                               .ptr = (uint8_t *)out_id_heap,
                               .len = strlen(out_id_heap),
                               .cap = strlen(out_id_heap) + 1,
                           },
                           .data = processCloud()};
    DoraResult_t send_result = (send_output->send_output.call)(
        send_output->send_output.env_ptr,
        output);  
    const char* a = "a";
    const uint8_t* b ;
    DoraResult_t send_result = dora_send_operator_output(send_output,a,b,1);

    OnEventResult_t result = { .result = send_result, .status = DORA_STATUS_CONTINUE };

    return result;
  }
  if (event->stop)
  {
    op->driver.stop();
    printf("C operator received stop event\n");
  }

  OnEventResult_t result = { .status = DORA_STATUS_CONTINUE };
  return result;
}

int main(){
  RawEvent_t mainEvent;
  mainEvent.stop=false;
  // mainEvent.input->id.len=0;
  SendOutput_t *out;
  DoraInitResult_t r=dora_init_operator();
  OnEventResult_t result;
    for(int i=0;i<10;++i){
      result= dora_on_event(//事件函数，一个很重要的问题是搞清楚里面的三个形参分别是干什么的
    &mainEvent,             //应该是输入事件，
    out,
    r.operator_context);
    //delete result.result.
    }
    return 0;
}
