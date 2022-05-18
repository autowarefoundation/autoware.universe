#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_

#include "modularized_particle_filter/correction/syncrho_subscriber.hpp"

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/cloud_with_pose.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <boost/circular_buffer.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <optional>

class CameraPoseCorrector : public rclcpp::Node
{
public:
  CameraPoseCorrector() : Node("camera_pose_corrector")
  {
    using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;

    // Publisher
    weighted_particle_pub_ = this->create_publisher<ParticleArray>("camera/weighted_particles", 10);
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/match_image", 10);

    // Subscriber
    synchro_subscriber_ = std::make_shared<SyncroSubscriber>(
      rclcpp::Node::SharedPtr{this}, "/lsd_cloud", "/ll2_cloud", "/predicted_particles");
    synchro_subscriber_->setCallback(
      std::bind(&CameraPoseCorrector::syncrhoCallback, this, _1, _2, _3));
  }

private:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using CloudPose = modularized_particle_filter_msgs::msg::CloudWithPose;

  // Subscriber
  std::shared_ptr<SyncroSubscriber> synchro_subscriber_;
  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  // Publisher
  rclcpp::Publisher<ParticleArray>::SharedPtr weighted_particle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  void syncrhoCallback(
    const PointCloud2 & msg1, const CloudPose & msg2, const ParticleArray & particles);
  void publishImage(const cv::Mat & image, const rclcpp::Time & stamp);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__CAMERA_POSE_CORRECTOR_HPP_
