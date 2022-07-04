#pragma once

#include <Eigen/StdVector>
#include <modularized_particle_filter/correction/abst_corrector.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace particle_filter
{
class GnssParticleCorrector : public AbstCorrector
{
public:
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Float32 = std_msgs::msg::Float32;

  GnssParticleCorrector();

private:
  rclcpp::Subscription<Float32>::SharedPtr height_sub_;
  rclcpp::Subscription<NavPVT>::SharedPtr ublox_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  const float flat_radius_;
  const float min_prob_;
  const float sigma_;
  const float float_range_gain_;
  Float32 latest_height_;

  void ubloxCallback(const NavPVT::ConstSharedPtr ublox_msg);
  rclcpp::Time ubloxTime2Stamp(const NavPVT & msg);

  ParticleArray weightParticles(
    const ParticleArray & predicted_particles, const Eigen::Vector3f & pose, bool fixed);

  float normalPdf(float x, float mu, float sigma);
  float inversePdf(float prob, bool fixed) const;

  void publishMarker(const Eigen::Vector3f & position, bool fixed);
};
}  // namespace particle_filter