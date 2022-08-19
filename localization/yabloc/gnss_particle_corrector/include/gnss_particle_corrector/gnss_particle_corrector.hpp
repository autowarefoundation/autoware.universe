
#ifndef GNSS_PARTILCE_CORRECTOR__GNSS_PARTICLE_CORRECTOR_HPP_
#define GNSS_PARTILCE_CORRECTOR__GNSS_PARTICLE_CORRECTOR_HPP_

#include <eigen3/Eigen/StdVector>
#include <modularized_particle_filter/correction/abst_corrector.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace modularized_particle_filter
{
class GnssParticleCorrector : public modularized_particle_filter::AbstCorrector
{
public:
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Float32 = std_msgs::msg::Float32;

  GnssParticleCorrector();

private:
  rclcpp::Subscription<Float32>::SharedPtr height_sub_;
  rclcpp::Subscription<NavPVT>::SharedPtr ublox_sub_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  const float float_range_gain_;
  const float likelihood_min_weight_;
  const float likelihood_stdev_;
  const float likelihood_flat_radius_;  //
  const bool rtk_enabled_;              // If false, all ublox_msgs are assumed FLOAT
  Float32 latest_height_;

  Eigen::Vector3f last_mean_position_;

  void onUblox(const NavPVT::ConstSharedPtr ublox_msg);
  void onPose(const PoseCovStamped::ConstSharedPtr pose_msg);

  ParticleArray weightParticles(
    const ParticleArray & predicted_particles, const Eigen::Vector3f & pose, float sigma,
    float flat_radius);

  float normalPdf(float x, float mu, float sigma);
  float inverseNormalPdf(float prob, bool fixed) const;

  void publishMarker(const Eigen::Vector3f & position, bool fixed);
};
}  // namespace modularized_particle_filter

#endif  // GNSS_PARTILCE_CORRECTOR__GNSS_PARTICLE_CORRECTOR_HPP_