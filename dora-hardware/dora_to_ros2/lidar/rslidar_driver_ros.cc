
#include "dora-node-api.h"
#include "dora-ros2-bindings.h"

#include <iostream>
#include <vector>
#include <random>

// rs lidar driver
#include <rs_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <rs_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <rs_driver/msg/point_cloud_msg.hpp>
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
    usleep(1000*100);

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


typedef struct Vec_uint8 {
    /** <No documentation available> */
    uint8_t * ptr;

    /** <No documentation available> */
    size_t len;

    /** <No documentation available> */
    size_t cap;
} Vec_uint8_t;
int main()
{
    std::cout << "rslidar driver for dora " << std::endl;

    RSDriverParam param;                  ///< Create a parameter object
    param.input_type = InputType::PCAP_FILE;
    param.input_param.pcap_path = "lidar.pcap";  ///< Set the pcap file directory
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
    driver.start();  ///< The driver thread will start



    auto dora_node = init_dora_node();
    auto merged_events = dora_events_into_combined(std::move(dora_node.events));

    auto qos = qos_default();

    qos.durability = Ros2Durability::Volatile;
    qos.liveliness = Ros2Liveliness::Automatic;
    qos.reliable = true;
    qos.lease_duration = 1;
    qos.max_blocking_time = 0.5;
    qos.keep_all = false;

    auto ros2_context = init_ros2_context();
    auto node = ros2_context->new_node("/ros2_demo", "turtle_teleop");
    auto vel_topic = node->create_topic_sensor_msgs_PointCloud2("/turtle1", "cmd_vel", qos);
    auto vel_publisher = node->create_publisher(vel_topic, qos);

    to_exit_process = false;

    while(!to_exit_process)
    {
        // copy from rslidar driver
        #if 1
        Vec_uint8_t result;
        rust::Vec<::std::uint8_t> dora_point;
        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();//这个popwait函数是一个线程安全的队列pop
        if (msg.get() == NULL)
        {
            std::cout<< "no msg"<<std::endl;
            continue;
        }
        // Well, it is time to process the point cloud msg, even it is time-consuming.
        RS_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << RS_REND;
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




        // std::cout << "HELLO FROM C++" << std::endl;





        // 获取当前时间点
        auto now = std::chrono::system_clock::now();
        // 将时间点转换为秒和纳秒部分
        auto now_sec = std::chrono::time_point_cast<std::chrono::seconds>(now);
        auto now_nsec = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        // 计算秒和纳秒
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(now_sec.time_since_epoch()).count();
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now_nsec.time_since_epoch()).count() % 1000000000;


        sensor_msgs::PointCloud2 PointCloud2;

        //将数据通过消息发送出去
        std::cout<< "ros2 topic begin "<<std::endl;
        PointCloud2.header.stamp.sec = sec;
        PointCloud2.header.stamp.nanosec = nsec;
        PointCloud2.header.frame_id = "rslidar";//帧id
        PointCloud2.height = 1;
        PointCloud2.width = output_data_len/16-1;
        sensor_msgs::PointField field1;
        field1.name = "x";
        field1.offset = 0;
        field1.datatype = 7;
        field1.count = 1;
        PointCloud2.fields.push_back(field1);
        sensor_msgs::PointField field2;
        field2.name = "y";
        field2.offset = 4;
        field2.datatype = 7;
        field2.count = 1;
        PointCloud2.fields.push_back(field2);
        sensor_msgs::PointField field3;
        field3.name = "z";
        field3.offset = 8;
        field3.datatype = 7;
        field3.count = 1;
        PointCloud2.fields.push_back(field3);
        sensor_msgs::PointField field4;
        field4.name = "i";
        field4.offset = 12;
        field4.datatype = 7;
        field4.count = 1;
        PointCloud2.fields.push_back(field4);

        PointCloud2.is_bigendian = false;
        PointCloud2.point_step = 16;
        PointCloud2.row_step = output_data_len/16-1;
        PointCloud2.is_dense =false;
        // PointCloud2.data.assign(output_data + 16, output_data + output_data_len);
        // PointCloud2.data.reserve(PointCloud2.data.size() + output_data_len - 16);
        std::copy(output_data + 16, output_data + output_data_len, std::back_inserter(PointCloud2.data));
       
        // PointCloud2.data = output_data;
        // std::copy(output_data+16, output_data + output_data_len, PointCloud2.data);
        // std::vector 
        // std::copy(output_data.begin(), output_data.end(), PointCloud2.data);
        // std::memcpy(PointCloud2.data.data(), output_data+16, output_data_len-16);
        // for (int i = 16; i<output_data_len; i++) {
        //     PointCloud2.data.push_back(static_cast<uint8_t>(output_data[i]));
        // }
        
        delete []output_data;
        output_data = NULL;
        vel_publisher->publish(PointCloud2);
    }
    to_exit_process = true;
    driver.stop();
       

   

    std::cout << "exit rslidar driver ..." << std::endl;

    return 0;
}

