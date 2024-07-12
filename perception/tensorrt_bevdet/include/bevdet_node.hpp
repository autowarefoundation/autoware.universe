#include <cstdio>
#include <string>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <cassert>
#include <map>

#include <yaml-cpp/yaml.h>
#include "bevdet.hpp"
#include "cpu_jpegdecoder.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>



#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>  // msg2pcl

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


typedef pcl::PointXYZI PointT;

uint8_t getSemanticType(const std::string & class_name);
void Getinfo(void);

void box3DToDetectedObjects(const std::vector<Box>& boxes, 
                    autoware_perception_msgs::msg::DetectedObjects & objects,
                    const std::vector<std::string> & class_names,  
                    float score_thre,
                    const bool has_twist);


// opencv Mat-> std::vector<char>
int cvToArr(cv::Mat img, std::vector<char> &raw_data)
{
    if (img.empty())
    {
        std::cerr << "image is empty. " << std::endl;
        return EXIT_FAILURE;
    }
    
    std::vector<u_char> raw_data_;
    cv::imencode(".jpg", img, raw_data_);
    raw_data = std::vector<char>(raw_data_.begin(), raw_data_.end());
    return EXIT_SUCCESS;
}

int cvImgToArr(std::vector<cv::Mat> &imgs, std::vector<std::vector<char>> &imgs_data)
{
    imgs_data.resize(imgs.size());

    for(size_t i = 0; i < imgs_data.size(); i++)
    {   
        if(cvToArr(imgs[i], imgs_data[i]))
        {
            return EXIT_FAILURE;
        }
    }
    return EXIT_SUCCESS;
}
 
class TRTBEVDetNode : public rclcpp::Node
{
private:

    size_t img_N_;
    int img_w_; 
    int img_h_;
    
    std::string model_config_;
    
    std::string onnx_file_;
    std::string engine_file_;
   
    YAML::Node camconfig_; 
    
    std::vector<std::string> imgs_name_;
    std::vector<std::string> class_names_;

    camsData sampleData_;
    std::shared_ptr<BEVDet> bevdet_;

    uchar* imgs_dev_ = nullptr; 
    float score_thre_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr pub_boxes_;//ros2无该消息类型
    
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_f_img_; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_fl_img_; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_fr_img_; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_b_img_; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_bl_img_; 
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_br_img_; 

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, 
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image,
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> MySyncPolicy;
    
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

public:
    TRTBEVDetNode(const std::string & node_name, const rclcpp::NodeOptions & options);
    ~TRTBEVDetNode();

    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg_cloud, 
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_f_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_fr_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_bl_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_b_img,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_br_img);
};