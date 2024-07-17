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
#include <unistd.h> // 在Linux/Mac下需要包含这个头文件
// 或
#include <thread>
#include <chrono>
#include <mutex>
 

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
//#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>
#include "cluster.h"
#include "boxer.h"
#include <Eigen/Core>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
 #include <pcl/filters/voxel_grid.h>
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


#include <nlohmann/json.hpp>
using json = nlohmann::json;



using namespace std;
using PointType = pcl::PointXYZI;

PolarGridBase *PolarGridBasor;
BoundingBoxCalculator *boxer;
Track *Trackor;
Eigen::Matrix3f car_pose_;
cv::Mat obj_image;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show(new pcl::PointCloud<pcl::PointXYZRGB>());
std::mutex mtx_pointcloud_show;

static Vec_uint8_t point_data;
bool to_exit_process = false;
uint32_t  local_cnt=0;//节点内部统计接收到数据的帧数量
 
// std::vector<double> lane_start_sd, lane_end_sd;


void* pointcloud_show_pthread(void *dora_context)
{
    const int rate = 50;   // 设定频率为 xx HZ
    const chrono::milliseconds interval((int)(1000/rate));
    //pcl::visualization::CloudViewer pcl_viewer("viewer");
    //pcl::visualization::PCLVisualizer::Ptr viewer; // (new pcl::visualization::PCLVisualizer ("3D Viewer"));
 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cluster_cloud"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "sample");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    // 创建可视化对象
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud_show, "sample");         
 
    while(!viewer->wasStopped())
    {   
        viewer->spinOnce(10);
        this_thread::sleep_for(interval);
    }
}

// 发布聚类以后的数据
int  dora_pub_tracker(custom_msgs::LidarRawObjectArray& LidarRawObjects,void *dora_context)
{
    // 创建一个空的 JSON 对象
    json j;
    nlohmann::json jsonArray_LidarRawObjects_objs = nlohmann::json::array();
    for(int n=0;n<LidarRawObjects.objs.size();n++)
    {
        custom_msgs::LidarRawObject LidarRawObject = LidarRawObjects.objs[n];

        json obj;

        nlohmann::json jsonArray_bbox_point_x = nlohmann::json::array();
        nlohmann::json jsonArray_bbox_point_y = nlohmann::json::array();
        nlohmann::json jsonArray_bbox_point_z = nlohmann::json::array();
        for(int i = 0; i < 8 ; i++)
        {
            jsonArray_bbox_point_x.push_back(LidarRawObject.bbox_point[i].x);
            jsonArray_bbox_point_y.push_back(LidarRawObject.bbox_point[i].y);
            jsonArray_bbox_point_z.push_back(LidarRawObject.bbox_point[i].z);
        }

        obj["bbox_point"]["x"] = jsonArray_bbox_point_x;
        obj["bbox_point"]["y"] = jsonArray_bbox_point_y;
        obj["bbox_point"]["z"] = jsonArray_bbox_point_z;

        obj["lwh"]["x"] = LidarRawObject.lwh.x;
        obj["lwh"]["y"] = LidarRawObject.lwh.y;
        obj["lwh"]["z"] = LidarRawObject.lwh.z;

        obj["x_pos"] = LidarRawObject.x_pos;
        obj["y_pos"] = LidarRawObject.y_pos;
        obj["z_pos"] = LidarRawObject.z_pos;

        jsonArray_LidarRawObjects_objs.push_back(obj);
    }
 
    j["LidarRawObjects"]["objs"] = jsonArray_LidarRawObjects_objs;
    j["header"]["frame_id"] = LidarRawObjects.head.frame_id;
    j["header"]["cnt"] = LidarRawObjects.head.seq;
    j["header"]["stamp"] = LidarRawObjects.head.stamp;
      
    // 将 JSON 对象序列化为字符串
    std::string json_string = j.dump(4); // 参数 4 表示缩进宽度
    //cout<<"======================================"<<endl;
    //cout<<json_string<<endl;
    // std::cout << "x: " << j["x"] << std::endl;
    // std::cout << "y: " << j["y"] << std::endl;
    // std::cout << "z: " << j["z"] << std::endl;


    // 将字符串转换为 char* 类型
    char *c_json_string = new char[json_string.length() + 1];
    strcpy(c_json_string, json_string.c_str());
    std::string out_id = "LidarRawObject";
    // std::cout<<json_string;
    int result = dora_send_output(dora_context, &out_id[0], out_id.length(), c_json_string, std::strlen(c_json_string));
    if (result != 0)
    {
        std::cerr << "LidarRawObject: failed to send output" << std::endl;
    }
    //std::cout << "dora_send_output()" << std::endl;

    return result;
}
int dora_pub_rgb_pointcloud(const pcl::PointCloud<pcl::PointXYZRGB> &points_msg,void *dora_context)
{
    size_t all_size = 16 + points_msg.size() * 16;
    point_data.ptr = new uint8_t[all_size];
     
    uint32_t* seq_ptr = (uint32_t*)point_data.ptr;
    *seq_ptr = points_msg.header.seq;

    double* timestamp_ptr = (double*)(point_data.ptr + 8);
    *timestamp_ptr = points_msg.header.stamp;

    // 直接这样内存转换会导致数据点丢失
    //memcpy(point_data.ptr + 16, &points_msg.points[0], points_msg.size() * 16);
   
    for(int i=0;i<points_msg.points.size();i++)
    {
        float* data_float = (float*)(point_data.ptr + 16+16*i);
        *data_float = points_msg.points[i].x;
        data_float = (float*)(point_data.ptr + 16+4+16*i);
        *data_float = points_msg.points[i].y;
        data_float = (float*)(point_data.ptr + 16+8+16*i);
        *data_float = points_msg.points[i].z;

        uint32_t* data_uint32 = (uint32_t*)(point_data.ptr + 16+12+16*i);
        *data_uint32 =( points_msg.points[i].r<<16 +  points_msg.points[i].g<<8+ points_msg.points[i].b);

    }

    char *output_data = (char *)point_data.ptr;
    size_t output_data_len = ((points_msg.size() + 1) * 16);
    std::string out_id = "rgb_pointcloud";
    std::cout << "euclidean_cluster output_data_len: " << output_data_len << std::endl;
    int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
    delete[] point_data.ptr;

    if (resultend != 0)
    {
        std::cerr << "pointcloud failed to send output" << std::endl;
        return 1;
    }
    return resultend;
}
int dora_pub_pointcloud(const pcl::PointCloud<PointType> &points_msg,void *dora_context)
{
    size_t all_size = 16 + points_msg.size() * 16;
    point_data.ptr = new uint8_t[all_size];
     
    uint32_t* seq_ptr = (uint32_t*)point_data.ptr;
    *seq_ptr = points_msg.header.seq;

    double* timestamp_ptr = (double*)(point_data.ptr + 8);
    *timestamp_ptr = points_msg.header.stamp;

    // 直接这样内存转换会导致数据点丢失
    //memcpy(point_data.ptr + 16, &points_msg.points[0], points_msg.size() * 16);
   
    for(int i=0;i<points_msg.points.size();i++)
    {
        float* data_float = (float*)(point_data.ptr + 16+16*i);
        *data_float = points_msg.points[i].x;
        data_float = (float*)(point_data.ptr + 16+4+16*i);
        *data_float = points_msg.points[i].y;
        data_float = (float*)(point_data.ptr + 16+8+16*i);
        *data_float = points_msg.points[i].z;
        data_float = (float*)(point_data.ptr + 16+12+16*i);
        *data_float =points_msg.points[i].intensity;

    }

    char *output_data = (char *)point_data.ptr;
    size_t output_data_len = ((points_msg.size() + 1) * 16);
    std::string out_id = "rgb_pointcloud";
    std::cout << "euclidean_cluster output_data_len: " << output_data_len << std::endl;
    int resultend = dora_send_output(dora_context, &out_id[0], out_id.length(), output_data, output_data_len);
    delete[] point_data.ptr;

    if (resultend != 0)
    {
        std::cerr << "pointcloud failed to send output" << std::endl;
        return 1;
    }
    return resultend;
}

int Callback(const pcl::PointCloud<PointType> &points_msg,void *dora_context)
{
 
    //pcl::console::TicToc tt;
    //tt.tic();
    double frame_time =  static_cast<double>(points_msg.header.stamp) / 1e9;//转化为时间
    std::cout.precision(20);
    uint32_t seq = points_msg.header.seq;
 
    pcl::PointCloud<PointType>::Ptr noground_cloud(new pcl::PointCloud<PointType>);    

    // pcl::fromROSMsg(*points_msg, *noground_cloud);
    noground_cloud = points_msg.makeShared();
    std::vector<pcl::PointCloud<PointType> > clusters;
    PolarGridBasor->polarGridCluster(noground_cloud, clusters);
  

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
        tmp_track.num_points = clusters[i].points.size();
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


 
    // step1:  这里需要发布LidarRawObjects 
    // pub_LidarRawObject.publish(LidarRawObjects);
    if(clusters.size()!=0)
    {
        dora_pub_tracker(LidarRawObjects,dora_context);
        std::cout<<"pub_LidarRawObject:  "<<LidarRawObjects.objs.size()<<std::endl;
    }
    else
    {
        std::cout<<"pub_LidarRawObject faild:  clusters.size()== 0"<<std::endl;
    }
   
    // for (int i = 0; i < trackers.size(); ++i)
    // {
    //     cout<<"ID: "<<i<<" center: "<<trackers[i].center[0]<<" "<<trackers[i].center[1]
    //         <<" size: "<<trackers[i].size.x<<"  "<<trackers[i].size.y<<"  "<<trackers[i].size.z<<"  "<<endl;
    // }



    // step2:  这里需要发布 点云
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;   
    PolarGridBasor->clusterstoColor(clusters, color_cloud);

 
    //-----------------------------------------------------------------------------------------
    std::cout <<"pub_data: input_cloud/color_cloud size():"<<points_msg.points.size()<<"/"<<color_cloud.points.size()<<std::endl;
    /*****这里需要发布color_cloud，未移植*****/
    // sensor_msgs::PointCloud2 output_points;
    // pcl::toROSMsg(color_cloud, output_points);
    // output_points.header.frame_id = "rslidar";
    // points_pub.publish(output_points);
    //int res = dora_pub_pointcloud(points_msg,dora_context);
    int res = dora_pub_rgb_pointcloud(color_cloud,dora_context);

    //std::cout << "the cost time of one frame is " << tt.toc() << std::endl;

    // std::vector<pcl::PointCloud<pcl::PointXYZI> >().swap(clusters);
    // std::vector<BoundingBoxCalculator::BoundingBox>().swap(boxes);
    return res;
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

    PolarGridBasor = new PolarGridBase();
    Trackor = new Track();
    boxer = new BoundingBoxCalculator;
    boxer->init();
 
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
            cout<<"\033[1;32m" << "id: "  << id << " id_len:"<<id_len << "\033[0m"<<endl;
            if (strncmp(id, "pointcloud",10) == 0)
            {
                char *data;
                size_t data_len;
                read_dora_input_data(event, &data, &data_len);
                cout<<"\033[1;32m" << "read data_len: "  <<data_len << "\033[0m"<<endl;
                int32_t point_len = (data_len-16)/4 ;
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
              

               // pointcloud_ptr = pointcloud.makeShared();

                std::cout.precision(20);
                cout<< "Euclidean Cluster Node Input PointCloud: " 
                    << " seq:"<<pointcloud.header.seq
                    << " stamp:"<<pointcloud.header.stamp/1e9
                    << " point.size:"<<pointcloud.points.size()<<endl;
                struct timeval tv;
                gettimeofday(&tv, NULL);
                cout << "The local" <<"  local_cnt: " <<local_cnt<<" recived data time: " << tv.tv_sec <<","<< tv.tv_usec/1000.0f<<" ms " 
                        << endl;
                local_cnt++;

                int ret_v = Callback(pointcloud,dora_context);
                //int ret_v = dora_pub_pointcloud(pointcloud,dora_context);
                

                pointcloud.clear();
                // pointcloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);

                std::cout << "完成计算一次" << std::endl;
                if (ret_v != 0)
                {
                    std::cerr << "failed to send output" << std::endl;
                    return 1;
                }
                free_dora_event(event);
            }
        }
        else if (ty == DoraEventType_Stop)
        {
            printf("euclidean_cluster: [c node] received stop event\n");
            free_dora_event(event);
        }
        else
        {
            printf("euclidean_cluster: [c node] received unexpected event: %d\n", ty);
            free_dora_event(event);
        }
      

    }
    return 0;
}

int main()
{
    std::cout << "euclidean_cluster for dora " << std::endl; 

    auto dora_context = init_dora_context_from_env();


    // pthread_t id = 0;
    // // 开启点云显示
    // if (pthread_create(&id, nullptr, pointcloud_show_pthread, dora_context) != 0)
    // {
    //     std::cerr << "create pointcloud_show_pthread thread fail!" << std::endl;
    //     exit(-1);
    // }

    auto ret = run(dora_context);
    free_dora_context(dora_context);

    to_exit_process = true;

    std::cout << "exit euclidean_cluster ..." << std::endl;
    return ret;
}
