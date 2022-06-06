#ifndef SIGN_DETECTOR__LL2_TO_IMAGE_HPP_
#define SIGN_DETECTOR__LL2_TO_IMAGE_HPP_
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/cloud_with_pose.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Ll2ImageConverter : public rclcpp::Node
{
public:
  Ll2ImageConverter();

private:
  const int line_thick_;
  const int image_size_;
  const float max_range_;  // [m]

  using CloudWithPose = modularized_particle_filter_msgs::msg::CloudWithPose;

  pcl::PointCloud<pcl::PointNormal>::Ptr linestrings_ = nullptr;

  rclcpp::Publisher<CloudWithPose>::SharedPtr pub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_height_;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
  cv::Mat lut_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  void poseCallback(const geometry_msgs::msg::PoseStamped & pose_stamped);

  void publishImage(const cv::Mat & image, const rclcpp::Time & stamp);
  void publishCloud(
    const pcl::PointCloud<pcl::PointNormal> & cloud, const rclcpp::Time & stamp,
    const geometry_msgs::msg::Pose & pose);

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);

  void makeDistanceImage(const cv::Mat & image);
  float computeHeight(const Eigen::Vector3f & pose);
};

#endif  // SIGN_DETECTOR__LL2_TO_IMAGE_HPP_