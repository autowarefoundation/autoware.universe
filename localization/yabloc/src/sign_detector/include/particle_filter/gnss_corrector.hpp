#pragma once

#include <Eigen/StdVector>
#include <modularized_particle_filter/correction/abst_corrector.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace particle_filter
{
class GnssParticleCorrector : public AbstCorrector
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Pose = geometry_msgs::msg::Pose;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  GnssParticleCorrector();

private:
  // rclcpp::Subscription<NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<NavPVT>::SharedPtr ublox_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  const float flat_radius_;
  const float min_prob_;
  const float sigma_;

  void fixCallback(const NavSatFix::ConstSharedPtr fix_msg);
  void ubloxCallback(const NavPVT::ConstSharedPtr ublox_msg);
  rclcpp::Time ubloxTime2Stamp(const NavPVT & msg);

  ParticleArray weightParticles(
    const ParticleArray & predicted_particles, const Eigen::Vector3f & pose);

  float normalPDF(float x, float mu, float sigma);

  void publishMarker(const Eigen::Vector3f & position);

  float inversePdf(float prob) const;
};
}  // namespace particle_filter