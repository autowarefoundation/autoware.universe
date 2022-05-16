#ifndef MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_

#include "modularized_particle_filter/prediction/resampler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <std_msgs/msg/float32.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>

class Predictor : public rclcpp::Node
{
public:
  Predictor();

private:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // Subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<ParticleArray>::SharedPtr weighted_particles_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;

  // Publisher
  rclcpp::Publisher<ParticleArray>::SharedPtr predicted_particles_pub_;
  rclcpp::Publisher<ParticleArray>::SharedPtr resampled_particles_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  const int number_of_particles_;
  const float resampling_interval_seconds_;
  float ground_height_;

  std::shared_ptr<RetroactiveResampler> resampler_ptr_;
  std::optional<ParticleArray> particle_array_opt_;
  std::optional<geometry_msgs::msg::TwistWithCovarianceStamped> twist_opt_;

  void gnssposeCallback(const PoseWithCovarianceStamped::ConstSharedPtr initialpose);
  void initialposeCallback(const PoseWithCovarianceStamped::ConstSharedPtr initialpose);
  void twistCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist);
  void weightedParticlesCallback(const ParticleArray::ConstSharedPtr weighted_particles);
  void timerCallback();

  geometry_msgs::msg::Pose calculateMeanPose(const ParticleArray & particle_array);
  void publishMeanPose(const geometry_msgs::msg::Pose & mean_pose);
};

#endif  // MODULARIZED_PARTICLE_FILTER__PREDICTION__PREDICTOR_HPP_
