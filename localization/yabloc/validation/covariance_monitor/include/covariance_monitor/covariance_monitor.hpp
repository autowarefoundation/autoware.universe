#pragma once
#include <eigen3/Eigen/Dense>
#include <pcdless_common/synchro_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace vmvl_validation
{
class CovarianceMonitor : public rclcpp::Node
{
public:
  using String = std_msgs::msg::String;
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  CovarianceMonitor();

private:
  SynchroSubscriber<ParticleArray, PoseStamped>::SharedPtr synchro_subscriber_;
  rclcpp::Publisher<String>::SharedPtr pub_diagnostic_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_cov_stamped_;

  void particleAndPose(const ParticleArray & particles, const PoseStamped & pose);
  Eigen::Vector3f computeStd(
    const ParticleArray & array, const Eigen::Quaternionf & orientation) const;
  void publishPoseCovStamped(const PoseStamped & pose, const Eigen::Vector3f & covariance);
};

}  // namespace vmvl_validation