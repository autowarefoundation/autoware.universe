#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_

#include "modularized_particle_filter/correction/abst_corrector.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

class GNSSPoseCorrector : public AbstCorrector
{
public:
  GNSSPoseCorrector();

private:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;

  const float flat_radius_;

  void poseCallback(const PoseWithCovarianceStamped::ConstSharedPtr pose_msg);
  ParticleArray calculateWeightedParticles(
    const ParticleArray & predicted_particles, PoseWithCovarianceStamped pose);

  float normalPDF(float x, float mu, float sigma);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
