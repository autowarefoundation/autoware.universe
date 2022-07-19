#pragma once
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <std_msgs/msg/string.hpp>

namespace validation
{

class CovarianceMonitor : public rclcpp::Node
{
public:
  using String = std_msgs::msg::String;
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;

  CovarianceMonitor();

private:
  rclcpp::Subscription<ParticleArray>::SharedPtr sub_particle_;
  rclcpp::Publisher<String>::SharedPtr pub_diagnostic_;

  void particleCallback(const ParticleArray & array);
  Eigen::Vector3f computeEigens(const ParticleArray & array) const;
};

}  // namespace validation