#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

extern "C"
{
#include "node_api.h"   
#include "operator_api.h"
#include "operator_types.h"
}

#include <iostream>
#include <vector>
#include <signal.h>

//#include <ros/ros.h>
//#include <sensor_msgs/PointCloud2.h>
// ##include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
//#include <pcl/impl/point_types.hpp>
#include "patchworkpp/patchworkpp.hpp"

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
//#include<custom_msgs/ErrorCode.h>

//typedef PointXYZI PointT;  
using PointType = pcl::PointXYZI;
using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

 
//ros::Publisher pub_view_status;

std::string lidar_deviceid;

bool to_exit_process = false;
static Vec_uint8_t point_data;
uint32_t local_cnt=0;// 统计该节点接收到多少条消息

int run(void *dora_context)
{
    if (dora_context == NULL)
    {
        fprintf(stderr, "failed to init dora context\n");
        return -1;
    }

    printf("[c node] dora context initialized\n");

    while(!to_exit_process)
    {
        void *event = dora_next_event(dora_context);
        if (event == NULL)
        {
            printf("[c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if (ty == DoraEventType_Input)
        {
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);
            if (strncmp(id, "pointcloud",10) == 0)
            {
               
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                // std::cout << "This is a dora ndt mapping node!" << std::endl;
                int32_t point_len = (data_len-16)/4 ;
                // std::cout << "data_len:" << data_len 
                //             << "  Point_len:" << point_len <<std::endl;

                pcl::PointCloud<pcl::PointXYZI> pointcloud;
                pointcloud.header.seq = *(std::uint32_t*)data;
                pointcloud.header.stamp = *(std::uint64_t*)(data+8);
                pointcloud.header.frame_id = "lidar";
                pointcloud.width = point_len;
                pointcloud.height = 1;
                pointcloud.is_dense = true;
                // pcl::PointXYZI* tem_point =  reinterpret_cast<pcl::PointXYZI*>(data.ptr + 16)
                // pointcloud.points = tem_point;
                for(int i = 0; i < (data_len-16)/16; i++){
                    pcl::PointXYZI tem_point;
                    
                    tem_point.x = *(float*)(data + 16 + 16 * i);
                    tem_point.y = *(float*)(data + 16 + 4 + 16 * i);
                    tem_point.z = *(float*)(data + 16 + 8 + 16 * i);
                    tem_point.intensity = *(float*)(data + 16 + 12 + 16 * i);
                    pointcloud.points.push_back(tem_point);
                    //   pointcloud.points.push_back(pcl::PointXYZI(*(float*)(data + 16 + 16 * i), *(float*)(data + 16 + 4 + 16 * i),*(float*)(data + 16 + 8 + 16 * i), *(float*)(data + 16 + 12 + 16 * i)));
                }
                std::cout.precision(20);
                cout<<"\033[1;32m" << "Input PointCloud: seq " << pointcloud.header.seq 
                    << "  stamp:" << pointcloud.header.stamp/1e9
                     << "\033[0m"<<endl;
                struct timeval tv;
                gettimeofday(&tv, NULL);
                cout << "The local" <<"  local_cnt: " <<local_cnt<<" recived data time: " << tv.tv_sec <<","<< tv.tv_usec/1000.0f<<" ms " 
                        << endl;
                local_cnt++;

                pointcloud_ptr = pointcloud.makeShared();

                // 调用点云去地面函数
                double time_taken;

                pcl::PointCloud<PointType> pc_curr;
                pcl::PointCloud<PointType> pc_ground;
                pcl::PointCloud<PointType> pc_non_ground;
                //custom_msgs::ErrorCode view_status;

                // pc_curr 表示输入点云
                PatchworkppGroundSeg->estimate_ground(pointcloud, pc_ground, pc_non_ground, time_taken);
                //if(pc_curr.size()<5000){
                //    view_status.deviceid=lidar_deviceid;
                //   view_status.error_type="lidar_field_view";
                //    view_status.is_error="0";
                //pub_view_status.publish(view_status); 
                //}
                cout<<"\033[1;32m" << "Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
                    << " (running_time: " << time_taken << " sec)" << "\033[0m"<<endl;
                

                //mapper.points_callback(pointcloud_ptr);
                pointcloud.clear();
                pointcloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
                
                // 发布 去地面以后的点云
                //// std::vector<Eigen::Vector4f> output_cloud;
                // std::cout << "output cloud size: " << output_cloud.size() << std::endl;

                size_t all_size = 16 + pc_non_ground.size() * 16;
                point_data.ptr = new uint8_t[all_size];

                memcpy(point_data.ptr, data, 16);
                //memcpy(point_data.ptr + 16, &pc_non_ground.points[0], pc_non_ground.size() * 16);

                for(int i=0;i<pc_non_ground.points.size();i++)
                {
                    float* data_float = (float*)(point_data.ptr + 16+16*i);
                    *data_float = pc_non_ground.points[i].x;
                    data_float = (float*)(point_data.ptr + 16+4+16*i);
                    *data_float = pc_non_ground.points[i].y;
                    data_float = (float*)(point_data.ptr + 16+8+16*i);
                    *data_float = pc_non_ground.points[i].z;
                    data_float = (float*)(point_data.ptr + 16+12+16*i);
                    *data_float =pc_non_ground.points[i].intensity;
                }
    

                char *output_data = (char *)point_data.ptr;
                size_t output_data_len = ((pc_non_ground.size() + 1) * 16);
                //size_t output_data_len = 16;
                std::string out_id = "pointcloud_no_ground";
                std::cout << "output_data_len: " << output_data_len << std::endl;
                int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
                delete[] point_data.ptr;
                if (resultend != 0)
                {
                    std::cerr << "failed to send output" << std::endl;
                    return 1;
                }
                free_dora_event(event);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            // mapper.saveMap();
            printf("[c node] received stop event\n");
            free_dora_event(event);
        }
        else
        {
            // mapper.saveMap();
            printf("[c node] received unexpected event: %d\n", ty);
            free_dora_event(event);
        }
    }

}
int main()
{
    std::cout << "patchwork-plusplus  for dora " << std::endl;

    std::cout << "Operating patchwork++..." << std::endl;

    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>());
 
 
    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);
 
 
    to_exit_process = true;
   
    std::cout << "exit patchwork-plusplus ..." << std::endl;
    return ret;
}
