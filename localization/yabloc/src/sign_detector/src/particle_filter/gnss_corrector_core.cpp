#include "particle_filter/gnss_corrector.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <trajectory/fix2mgrs.hpp>

namespace particle_filter
{
GnssParticleCorrector::GnssParticleCorrector()
: AbstCorrector("gnss_particle_corrector"),
  flat_radius_(declare_parameter("flat_radius", 1.0f)),
  min_prob_(declare_parameter("min_prob", 0.01f)),
  sigma_(declare_parameter("sigma", 25.0f))
{
  using std::placeholders::_1;
  auto fix_callback = std::bind(&GnssParticleCorrector::fixCallback, this, _1);
  fix_sub_ = create_subscription<NavSatFix>("/sensing/gnss/ublox/nav_sat_fix", 10, fix_callback);

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
}

void GnssParticleCorrector::fixCallback(const NavSatFix::ConstSharedPtr fix_msg)
{
  const rclcpp::Time stamp = fix_msg->header.stamp;
  const int FIX_FLAG = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  if (fix_msg->status.status != FIX_FLAG) return;

  std::optional<ParticleArray> opt_particles = getSyncronizedParticleArray(stamp);
  if (!opt_particles.has_value()) return;

  auto dt = (stamp - rclcpp::Time(opt_particles->header.stamp));
  RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt.seconds());

  Eigen::Vector3f position = fix2Mgrs(*fix_msg).cast<float>();
  ParticleArray weighted_particles{weightParticles(opt_particles.value(), position)};
  setWeightedParticleArray(weighted_particles);
}

GnssParticleCorrector::ParticleArray GnssParticleCorrector::weightParticles(
  const ParticleArray & predicted_particles, const Eigen::Vector3f & pose)
{
  ParticleArray weighted_particles{predicted_particles};

  for (int i{0}; i < static_cast<int>(predicted_particles.particles.size()); i++) {
    float distance{static_cast<float>(std::hypot(
      predicted_particles.particles[i].pose.position.x - pose.x(),
      predicted_particles.particles[i].pose.position.y - pose.y()))};

    if (distance < flat_radius_) {
      weighted_particles.particles[i].weight = 1.0f;
    } else {
      weighted_particles.particles[i].weight = normalPDF(distance - flat_radius_, 0.0, sigma_);
    }

    weighted_particles.particles[i].weight =
      std::max(weighted_particles.particles[i].weight, min_prob_);
  }

  return weighted_particles;
}

float GnssParticleCorrector::normalPDF(float x, float mu, float sigma)
{
  // NOTE: This is not exact normal distribution
  float a = (x - mu) / sigma;
  return std::exp(-0.5f * a * a);
}

}  // namespace particle_filter