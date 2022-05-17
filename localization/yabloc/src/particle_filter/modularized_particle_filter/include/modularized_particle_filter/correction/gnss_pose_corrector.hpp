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
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // Publisher and subscriber
  rclcpp::Subscription<ParticleArray>::SharedPtr particle_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<ParticleArray>::SharedPtr weighted_particle_pub_;

  // Circular buffer
  const int particles_buffer_size_;
  const float flat_radius_;
  const int pose_buffer_size_;
  boost::circular_buffer<ParticleArray> particles_circular_buffer_;
  boost::circular_buffer<PoseWithCovarianceStamped> pose_circular_buffer_;

  void particleCallback(const ParticleArray::ConstSharedPtr particles);
  void poseCallback(const PoseWithCovarianceStamped::ConstSharedPtr pose_msg);
  void correctAndPublishParticles();
  ParticleArray calculateWeightedParticles(
    ParticleArray predicted_particles, PoseWithCovarianceStamped pose, float flat_radius);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
