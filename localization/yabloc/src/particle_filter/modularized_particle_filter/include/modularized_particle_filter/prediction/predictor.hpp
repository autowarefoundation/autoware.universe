#ifndef MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_

#include "modularized_particle_filter/prediction/resampler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <iostream>
#include <memory>
#include <optional>

class Predictor : public rclcpp::Node
{
public:
  Predictor();

private:
  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    weighted_particles_sub_;

  // Publisher
  rclcpp::Publisher<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    predicted_particles_pub_;
  rclcpp::Publisher<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    resampled_particles_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  int number_of_particles_;
  float resampling_interval_seconds_;

  std::shared_ptr<RetroactiveResampler> resampler_ptr_;
  std::optional<modularized_particle_filter_msgs::msg::ParticleArray> particle_array_opt_;
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> twist_opt_;

  void initialposeCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initialpose);
  void twistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist);
  void weightedParticlesCallback(
    const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr weighted_particles);
  void timerCallback();

  geometry_msgs::msg::Pose calculateMeanPose(
    const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);
  void publishMeanPose(const geometry_msgs::msg::Pose & mean_pose);
};

#endif  // MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_
