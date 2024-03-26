extern "C"
{
#include "/home/char/dora_/dora-rs/dora/apis/c/node/node_api.h"   
#include "/home/char/dora_/dora-rs/dora/apis/c/operator/operator_api.h"
#include "/home/char/dora_/dora-rs/dora/apis/c/operator/operator_types.h"
}

#include <iostream>
#include <vector>


// rs lidar driver
#include </home/char/dora_/dora-autoware/dora-hardware/dora_to_ros2/lidar/rs_driver/src/rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include </home/char/dora_/dora-autoware/dora-hardware/dora_to_ros2/lidar/rs_driver/src/rs_driver/msg/point_cloud_msg.hpp>
#endif
typedef PointXYZI PointT;                       // x,y,z, intensity;
                                                // intensity: ji guang fan she qiang du  
                                                // this is a point in the point cloud

/// @brief PointCloudMsg
//  typedef std::vector<PointT> VectorT
//  uint32_t height = 0
//  uint32_t width = 0
//  bool is_dense = false
//  double timestamp = 0
//  uint32_t seq = 0  //sequence number of message
//  std::string frame_id = ""  
//  vectorT points
typedef PointCloudT<PointT> PointCloudMsg;       

using namespace robosense::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;
//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)//从free队列里面取
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller. 
// @param msg  The stuffed point cloud message.
//
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)//填到一个新的队列里
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
  stuffed_cloud_queue.push(msg);
}

//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
std::string exceptionCallback(const Error& code)//错误报告
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() << RS_REND;
  return "";
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();//这个popwait函数是一个线程安全的队列pop
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the point cloud msg, even it is time-consuming.
    RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

#if 0
    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
    {
      std::cout << std::fixed << std::setprecision(3) 
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                << std::endl;
    }
#endif

    free_cloud_queue.push(msg);//这里是说，上面那个#if里面的东西已经把这个点云处理完了，东西都取出来了，那这个点云实例（占内存的）我们就可以重复利用了，就空闲了，把它放入待使用区（free区）
  }
}
int run(void *dora_context)
{
    unsigned char counter = 0;

    //for (int i = 0; i < 20; i++)
    to_exit_process = false;
    while(!to_exit_process)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);

        if (ty == DoraEventType_Input)
        {
            // copy from rslidar driver
            #if 1
            Vec_uint8_t result;
            
                std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();//这个popwait函数是一个线程安全的队列pop
                if (msg.get() == NULL)
                {
                    continue;
                }

                // Well, it is time to process the point cloud msg, even it is time-consuming.
                RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;

                #if 0
                    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
                    {
                    std::cout << std::fixed << std::setprecision(3) 
                                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                                << std::endl;
                    }
                #endif

                //free_cloud_queue.push(msg);//这里是说，上面那个#if里面的东西已经把这个点云处理完了，东西都取出来了，那这个点云实例（占内存的）我们就可以重复利用了，就空闲了，把它放入待使用区（free区）
                
                if (sizeof(PointT) <= 16)
                {
                    RS_MSG << "sizeof(PointT) <= 16 " << RS_REND;
                    size_t cloudSize = (((msg->points.size()) + 1) * 16);  // 4byte for message seq, 4bytes empty, 8byte for timestamp,
                                                                        // others for points
                    u_int8_t* bytePointCloud = (u_int8_t*)(new PointT[cloudSize / sizeof(PointT)]);
                    
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
                    
                    result.ptr = bytePointCloud;
                    result.len = cloudSize;
                    result.cap = cloudSize;
                    //return result;
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
                    //Vec_uint8_t result;
                    result.ptr = bytePointCloud;
                    result.len = cloudSize;
                    result.cap = cloudSize;
                    //return result;
                }
                else
                {
                    std::cerr << "point size error! This may happen when your system is not byte aligned!";
                    result = { .ptr = NULL };
                    result.len = 0;
                    result.cap = 0;
                    //return result;
                }
            
            #endif
 
            char* output_data = (char *)result.ptr;
            size_t output_data_len = result.len;
            counter += 1;

            std::string out_id = "pointcloud";
            size_t data_len = 1 ;
            int resultend=dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
           
           std::cout
                << "dora_send_output: out_id "<<out_id<< "  out_data_len: "<<output_data_len<<std::endl;
                
            if (resultend != 0)
            {
                std::cerr << "failed to send output" << std::endl;
                return 1;
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("[c node] received stop event\n");
        }
        else
        {
            printf("[c node] received unexpected event: %d\n", ty);
        }

        free_dora_event(event);
    }
    return 0;
}

int main()
{
    std::cout << "rslidar driver for dora " << std::endl;

    RSDriverParam param;                  ///< Create a parameter object
    param.input_type = InputType::PCAP_FILE;
    param.input_param.pcap_path ="/home/char/ndt_localizer_new/input_points/lidar2.pcap";  ///< Set the pcap file directory
    param.input_param.msop_port = 6699;   ///< Set the lidar msop port number, the default is 6699
    param.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
    param.lidar_type = LidarType::RSHELIOS;   ///< Set the lidar type. Make sure this type is correct雷达类型
    param.print();//控制台输出参数信息

    LidarDriver<PointCloudMsg> driver;               ///< Declare the driver object
    driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); ///< Register the point cloud callback functions
    driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function
    if (!driver.init(param))                         ///< Call the init function
    {
        RS_ERROR << "Driver Initialize Error..." << RS_REND;
        return -1;
    }
    //std::thread cloud_handle_thread = std::thread(processCloud);

    driver.start();  ///< The driver thread will start
    RS_DEBUG << "RoboSense Lidar-Driver Linux online demo start......" << RS_REND;
 
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
 
    to_exit_process = true;
    driver.stop();
    std::cout << "exit rslidar driver ..." << std::endl;
    return ret;
}

