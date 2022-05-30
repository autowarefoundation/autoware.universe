#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_

#include "modularized_particle_filter/correction/syncrho_subscriber.hpp"

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/cloud_with_pose.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>

#include <optional>

class CameraPoseCorrector : public rclcpp::Node
{
public:
  CameraPoseCorrector()
  : Node("camera_pose_corrector"),
    image_size_(declare_parameter<int>("image_size", 800)),
    max_range_(declare_parameter<float>("max_range", 20.f)),
    score_threshold_(declare_parameter<float>("score_threshold", 10.f))
  {
    using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;

    // Publisher
    weighted_particle_pub_ = this->create_publisher<ParticleArray>("/weighted_particles", 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/match_image", 10);

    // Subscriber
    synchro_subscriber_ = std::make_shared<SyncroSubscriber>(
      rclcpp::Node::SharedPtr{this}, "/lsd_cloud", "/ll2_cloud", "/predicted_particles");
    synchro_subscriber_->setCallback(
      std::bind(&CameraPoseCorrector::synchroCallback, this, _1, _2, _3));

    lut_ = cv::Mat(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++) {
      lut_.at<uchar>(0, i) = 256 * std::pow(i / 256.f, 4.0f);
    }
  }

private:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CloudPose = modularized_particle_filter_msgs::msg::CloudWithPose;

  const int image_size_;
  const float max_range_;
  const float score_threshold_;

  // Subscriber
  std::shared_ptr<SyncroSubscriber> synchro_subscriber_;
  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  // Publisher
  rclcpp::Publisher<ParticleArray>::SharedPtr weighted_particle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  void synchroCallback(
    const PointCloud2 & msg1, const CloudPose & msg2, const ParticleArray & particles);

  void publishImage(const cv::Mat & image, const rclcpp::Time & stamp);
  cv::Mat dilateImage(const cv::Mat & image);
  cv::Mat cloud2Image(const pcl::PointCloud<pcl::PointNormal> & cloud);
  cv::Point2f toCvPoint(const Eigen::Vector3f & p);
  cv::Mat visualizePairs(const std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>> & pairs);

  cv::Mat buildLl2Image(const pcl::PointCloud<pcl::PointNormal> & cloud);

  pcl::PointCloud<pcl::PointNormal> rejectOutsideLines(
    const pcl::PointCloud<pcl::PointNormal> & cloud);
  cv::Mat lut_;

  std::pair<float, std::vector<std::pair<pcl::PointNormal, pcl::PointNormal>>> computeScore(
    const pcl::PointCloud<pcl::PointNormal> & lsd_cloud,
    std::unordered_map<int, std::vector<pcl::PointNormal>> & angle_and_lines);

  float computeScore(
    const pcl::PointCloud<pcl::PointNormal> & lsd_cloud, const cv::Mat & ll2_image);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_
