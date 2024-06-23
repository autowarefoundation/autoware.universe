extern "C"
{
#include "node_api.h"
#include "operator_api.h"
#include "operator_types.h"
}
#include <iostream>
#include <vector>
#include <cmath>
#include <sys/time.h>

#include <Eigen/Dense>
#include <cstdint>
#include <memory>
#include <string.h>
#include <cassert>
#include <string.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <msgpack.hpp> //sudo apt-get install libmsgpack-dev
// #include <dora-ros2-bindings.h>

#include <string>
#include <vector>
//#include <ros/ros.h>
//#include "ros/package.h"
#include <stdio.h>
#include <stack>
#include <math.h>
#include <time.h>
#include <pcl/common/common.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>
#include "cluster.h"
#include "boxer.h"
#include <Eigen/Core>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include <pcl/console/time.h>
#include "tracker.h"
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <image_transport/image_transport.h>
// #include <custom_msgs/LidarRawObject.h>
// #include <custom_msgs/LidarRawObjectArray.h>
#include "../include/msg.hpp"
// ros::NodeHandle *nh;
// ros::Publisher points_pub, raw_points_pub , pub_LidarRawObject;
// image_transport::Publisher outmap_pub;


using namespace std;
using PointType = pcl::PointXYZI;

PolarGridBase *PolarGridBasor;
BoundingBoxCalculator *boxer;
Track *Trackor;
Eigen::Matrix3f car_pose_;
cv::Mat obj_image;


static Vec_uint8_t point_data;
bool to_exit_process = false;
uint32_t  local_cnt=0;//节点内部统计接收到数据的帧数量

// std::vector<double> lane_start_sd, lane_end_sd;

int Callback(const pcl::PointCloud<PointType> &points_msg,void *dora_context)
{
 
    pcl::console::TicToc tt;
    double frame_time =  static_cast<double>(points_msg.header.stamp) / 1e9;//转化为时间
    std::cout.precision(20);
    uint32_t seq = points_msg.header.seq;
 
    pcl::PointCloud<PointType>::Ptr noground_cloud(new pcl::PointCloud<PointType>);    

    // pcl::fromROSMsg(*points_msg, *noground_cloud);
    noground_cloud = points_msg.makeShared();

    std::vector<pcl::PointCloud<PointType> > clusters;
    
    std::cout<<" flag3 "<<std::endl;
    PolarGridBasor->polarGridCluster(noground_cloud, clusters);
    std::cout<<" flag4 "<<std::endl;

    std::vector<BoundingBoxCalculator::BoundingBox> boxes;
    std::vector<tracker> trackers;
    custom_msgs::LidarRawObjectArray LidarRawObjects;

    //simple classify the object
    for (int i = 0; i < clusters.size(); ++i)
    {
        BoundingBoxCalculator::BoundingBox tmp_box = boxer->calcBoundingBox(clusters[i]);
       if (tmp_box.size.x * tmp_box.size.y > 30)
           continue;
        Eigen::Vector2f v1(tmp_box.center.x, tmp_box.center.y);
        float distance = v1.norm();
        // if (tmp_box.size.z > 2.5 && distance > 8)s
        //     continue;
       if (tmp_box.size.x / tmp_box.size.y > 4 && tmp_box.size.y < 0.1)
           continue;
       if (tmp_box.size.x < 0.4 && tmp_box.size.y < 0.4 && tmp_box.size.z < 0.4)
           continue;
       if(tmp_box.size.x < 0.1 || tmp_box.size.z < 0.1)
           continue;
        //boxes.push_back(tmp_box);
        tracker tmp_track;
        tmp_track.kalman_init();
        tmp_track.num_points = clusters[i].points   .size();
        tmp_track.center[0] = tmp_box.center.x;
        tmp_track.center[1] = tmp_box.center.y;
        tmp_track.center[2] = 0;
        tmp_track.size.x = tmp_box.size.x;
        tmp_track.size.y = tmp_box.size.y;
        tmp_track.size.z = tmp_box.size.z;
        tmp_track.corners[0] = tmp_box.corners[0];
        tmp_track.corners[1] = tmp_box.corners[1];
        tmp_track.corners[2] = tmp_box.corners[2];
        tmp_track.corners[3] = tmp_box.corners[3];
        tmp_track.corners[4] = tmp_box.corners[4];
        tmp_track.corners[5] = tmp_box.corners[5];
        tmp_track.corners[6] = tmp_box.corners[6];
        tmp_track.corners[7] = tmp_box.corners[7];
        tmp_track.latest_tracked_time = frame_time;
        trackers.push_back(tmp_track);
        custom_msgs::LidarRawObject LidarRawObject;
        for(int i = 0; i < 8 ; i++)
        {
            LidarRawObject.bbox_point[i].x = tmp_box.corners[i].x;
            LidarRawObject.bbox_point[i].y = tmp_box.corners[i].y;
            LidarRawObject.bbox_point[i].z = tmp_box.corners[i].z;
        }
        LidarRawObject.lwh.x = tmp_box.size.x;
        LidarRawObject.lwh.y = tmp_box.size.y;
        LidarRawObject.lwh.z = tmp_box.size.z;
        LidarRawObject.x_pos = tmp_box.center.x;
        LidarRawObject.y_pos = tmp_box.center.y;
        LidarRawObject.z_pos = tmp_box.center.z;
        LidarRawObjects.objs.push_back(LidarRawObject);
    }
    LidarRawObjects.head = points_msg.header;

    /*****这里需要发布LidarRawObjects，未移植*****/
    // pub_LidarRawObject.publish(LidarRawObjects);

    std::cout<<" PolarGridBasor->clusterstoColor() "<<frame_time<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    PolarGridBasor->clusterstoColor(clusters, color_cloud);
   
    std::cout << "publish data: " <<std::endl;
   
    /*****这里需要发布color_cloud，未移植*****/
    // sensor_msgs::PointCloud2 output_points;
    // pcl::toROSMsg(color_cloud, output_points);
    // output_points.header.frame_id = "rslidar";
    // points_pub.publish(output_points);

    // 发布 去地面以后的点云
    //// std::vector<Eigen::Vector4f> output_cloud;
    // std::cout << "output cloud size: " << output_cloud.size() << std::endl;

    size_t all_size = 16 + color_cloud.size() * 16;
    point_data.ptr = new uint8_t[all_size];

     
    uint32_t* seq_ptr = (uint32_t*)point_data.ptr;
    *seq_ptr = seq;
    double* timestamp_ptr = (double*)(point_data.ptr + 8);
    *timestamp_ptr = points_msg.header.stamp;
    memcpy(point_data.ptr + 16, &color_cloud.points[0], color_cloud.size() * 16);


    char *output_data = (char *)point_data.ptr;
    size_t output_data_len = ((color_cloud.size() + 1) * 16);
    //size_t output_data_len = 16;
    std::string out_id = "pointcloud_euclidean_cluster";
    std::cout << "euclidean_cluster output_data_len: " << output_data_len << std::endl;
    int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
    delete[] point_data.ptr;
    if (resultend != 0)
    {
        std::cerr << "failed to send output" << std::endl;
        return 1;
    }
//    std::cout << "the cost time of one frame is " << tt.toc() << std::endl;s

    std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(clusters);
    std::vector<BoundingBoxCalculator::BoundingBox>().swap(boxes);
}


/***object_cb函数，未移植****/
// void object_cb(const sensor_msgs::ImageConstPtr &image_msg)
// {
//     std::vector<tracker> new_objects;
//     Trackor->getObjects(new_objects);
//     Trackor->getCarPose(car_pose_);

//     const int FRONT_SCAN_RANGE = 250;      //前方扫描有效范围250
//     const int LEFT_SCAN_RANGE = 100;       //左方扫描有效范围100

//     obj_image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
//     cv::Point pt[1][4];

//     for(int i = 0;i<new_objects.size();++i)
//     {
//         //new_objects[i].center = car_pose_.inverse()*new_objects[i].center;
//         if(new_objects[i].center[0] > 20||new_objects[i].center[0] < -20)
//             continue;
//         if(new_objects[i].center[1] > 10||new_objects[i].center[1] < -10)
//             continue;
//         pt[0][0].y = FRONT_SCAN_RANGE - new_objects[i].corners[0].x/0.2;
//         pt[0][0].x = LEFT_SCAN_RANGE - new_objects[i].corners[0].y/0.2;
//         pt[0][1].y = FRONT_SCAN_RANGE - new_objects[i].corners[1].x/0.2; 
//         pt[0][1].x = LEFT_SCAN_RANGE - new_objects[i].corners[1].y/0.2;             
//         pt[0][2].y = FRONT_SCAN_RANGE - new_objects[i].corners[2].x/0.2;
//         pt[0][2].x = LEFT_SCAN_RANGE - new_objects[i].corners[2].y/0.2;
//         pt[0][3].y = FRONT_SCAN_RANGE - new_objects[i].corners[3].x/0.2; 
//         pt[0][3].x = LEFT_SCAN_RANGE - new_objects[i].corners[3].y/0.2;     

//         const cv::Point* ppt[1]={pt[0]};
//         int npt[] = {4};
//         cv::polylines(obj_image, ppt, npt, 1, 1, cv::Scalar(0,255,0),1,8,0);        
//         cv::fillPoly(obj_image, ppt, npt, 1, cv::Scalar(0,255,0));     //绘制四边形,并填充
//     }
//     //std::cout<<new_objects.size()<<std::endl;


//     // cv::namedWindow("object Image", cv::WINDOW_NORMAL);
//     // cv::imshow("object Image",obj_image);
//     // cv::waitKey(10);



//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", obj_image).toImageMsg();
//     outmap_pub.publish(msg);

// }



int run(void *dora_context)
{
    if (dora_context == NULL)
    {
        fprintf(stderr, "euclidean_cluster: failed to init dora context\n");
        return -1;
    }

    printf("euclidean_cluster: [c node] dora context initialized\n");
    while(!to_exit_process)
    {
        void *event = dora_next_event(dora_context);
      
        if (event == NULL)
        {
            printf("euclidean_cluster: [c node] ERROR: unexpected end of event\n");
            return -1;
        }

        enum DoraEventType ty = read_dora_event_type(event);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        if (ty == DoraEventType_Input)
        {
            //printf("flag1 \n");
            
            char *id;
            size_t id_len;
            read_dora_input_id(event, &id, &id_len);
             cout<<"\033[1;32m" << "id: "  << id
                    << " id_len:"<<id_len
                    << "\033[0m"<<endl;
            if (strcmp(id, "pointcloud") == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);

                int32_t point_len = (data_len-16)/4 ;
                pcl::PointCloud<pcl::PointXYZI> pointcloud;
                pointcloud.header.seq = *(std::uint32_t*)data;
                pointcloud.header.stamp = *(std::uint64_t*)(data+8);
                pointcloud.header.frame_id = "lidar";
                pointcloud.width = point_len;
                pointcloud.height = 1;
                pointcloud.is_dense = true;
                for(int i = 0; i < (data_len-16)/16; i++){
                    pcl::PointXYZI tem_point;
                    tem_point.x = *(float*)(data + 16 + 16 * i);
                    tem_point.y = *(float*)(data + 16 + 4 + 16 * i);
                    tem_point.z = *(float*)(data + 16 + 8 + 16 * i);
                    tem_point.intensity = *(float*)(data + 16 + 12 + 16 * i);
                    pointcloud.points.push_back(tem_point);
                }
                // pointcloud_ptr = pointcloud.makeShared();
                std::cout.precision(20);
                cout<<"\033[1;32m" << "Euclidean Cluster Node Input PointCloud: " 
                    << " seq:"<<pointcloud.header.seq
                    <<"  local_cnt: " <<local_cnt
                    << " stamp:"<<pointcloud.header.stamp/1e9
                    << " point.size:"<<pointcloud.points.size()<< "\033[0m"<<endl;
                
                local_cnt++;

                Callback(pointcloud,dora_context);
                pointcloud.clear();
                // pointcloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
                std::cout << "完成计算一次" << std::endl;
                
                free_dora_event(event);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("euclidean_cluster: [c node] received stop event\n");
        }
        else
        {
            printf("euclidean_cluster: [c node] received unexpected event: %d\n", ty);
        }
      

    }
    return 0;
}

int main()
{
    std::cout << "euclidean_cluster for dora " << std::endl; 

    auto dora_context = init_dora_context_from_env();
    auto ret = run(dora_context);
    free_dora_context(dora_context);

    to_exit_process = true;

    std::cout << "exit euclidean_cluster ..." << std::endl;
    return ret;
}
