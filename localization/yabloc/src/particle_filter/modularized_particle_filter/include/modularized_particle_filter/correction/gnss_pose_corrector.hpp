#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include <boost/circular_buffer.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <optional>
#include <vector>

class GNSSPoseCorrector : public rclcpp::Node
{
public:
  GNSSPoseCorrector();

private:
  rclcpp::Subscription<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    particle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<modularized_particle_filter_msgs::msg::ParticleArray>::SharedPtr
    weighted_particle_pub_;

  int particles_buffer_size_;
  float flat_radius_;
  boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray>
    particles_circular_buffer_;
  int pose_buffer_size_;
  boost::circular_buffer<geometry_msgs::msg::PoseWithCovarianceStamped> pose_circular_buffer_;

  void particleCallback(
    const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr particles);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg);

  void correctAndPublishParticles();
  modularized_particle_filter_msgs::msg::ParticleArray calculateWeightedParticles(
    modularized_particle_filter_msgs::msg::ParticleArray predicted_particles,
    geometry_msgs::msg::PoseWithCovarianceStamped pose, float flat_radius);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
