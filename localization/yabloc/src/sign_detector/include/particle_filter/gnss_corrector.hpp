#pragma once

#include <Eigen/StdVector>
#include <modularized_particle_filter/correction/abst_corrector.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace particle_filter
{
class GnssParticleCorrector : public AbstCorrector
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;
  using NavSatFix = sensor_msgs::msg::NavSatFix;

  GnssParticleCorrector();

private:
  rclcpp::Subscription<NavSatFix>::SharedPtr fix_sub_;

  const float flat_radius_;
  const float min_prob_;
  const float sigma_;

  void fixCallback(const NavSatFix::ConstSharedPtr fix_msg);

  ParticleArray weightParticles(
    const ParticleArray & predicted_particles, const Eigen::Vector3f & pose);

  float normalPDF(float x, float mu, float sigma);
};
}  // namespace particle_filter