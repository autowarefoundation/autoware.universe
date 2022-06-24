#include "particle_filter/gnss_corrector.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <common/util.hpp>
#include <trajectory/fix2mgrs.hpp>

namespace particle_filter
{
GnssParticleCorrector::GnssParticleCorrector()
: AbstCorrector("gnss_particle_corrector"),
  flat_radius_(declare_parameter("flat_radius", 1.0f)),
  min_prob_(declare_parameter("min_prob", 0.01f)),
  sigma_(declare_parameter("sigma", 25.0f)),
  float_range_gain_(2.0f)
{
  using std::placeholders::_1;
  auto ublox_callback = std::bind(&GnssParticleCorrector::ubloxCallback, this, _1);
  auto height_callback = [this](const Float32 & height) { this->latest_height_ = height; };

  ublox_sub_ = create_subscription<NavPVT>("/sensing/gnss/ublox/navpvt", 10, ublox_callback);
  height_sub_ = create_subscription<Float32>("/height", 10, height_callback);
  marker_pub_ = create_publisher<MarkerArray>("/gnss/effect_marker", 10);
}

void GnssParticleCorrector::ubloxCallback(const NavPVT::ConstSharedPtr ublox_msg)
{
  const rclcpp::Time stamp = util::ubloxTime2Stamp(*ublox_msg);

  const int FIX_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FIXED;
  const int FLOAT_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FLOAT;

  if (!(ublox_msg->flags & FIX_FLAG) & !(ublox_msg->flags & FLOAT_FLAG)) return;

  std::optional<ParticleArray> opt_particles = getSyncronizedParticleArray(stamp);
  if (!opt_particles.has_value()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "There is no corresponding opt_particles " << stamp.nanoseconds());
    return;
  }

  auto dt = (stamp - rclcpp::Time(opt_particles->header.stamp));
  if (std::abs(dt.seconds()) > 0.1) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between gnss and particles is LARGE " << dt.seconds());
  }

  NavSatFix fix;
  fix.latitude = ublox_msg->lat * 1e-7f;
  fix.longitude = ublox_msg->lon * 1e-7f;
  fix.altitude = ublox_msg->height * 1e-3f;

  const bool fixed = (ublox_msg->flags & FIX_FLAG);

  Eigen::Vector3f position = fix2Mgrs(fix).cast<float>();
  ParticleArray weighted_particles{weightParticles(opt_particles.value(), position, fixed)};
  setWeightedParticleArray(weighted_particles);

  publishMarker(position, fixed);
}

void GnssParticleCorrector::publishMarker(const Eigen::Vector3f & position, bool fixed)
{
  using namespace std::literals::chrono_literals;
  using Point = geometry_msgs::msg::Point;

  auto drawCircle = [](std::vector<Point> & points, float radius) -> void {
    const int N = 10;
    for (int theta = 0; theta < 2 * N + 1; theta++) {
      geometry_msgs::msg::Point pt;
      pt.x = radius * std::cos(theta * M_PI / N);
      pt.y = radius * std::sin(theta * M_PI / N);
      points.push_back(pt);
    }
  };

  MarkerArray array_msg;
  for (int i = 0; i < 5; i++) {
    Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "/map";
    marker.id = i;
    marker.type = Marker::LINE_STRIP;
    marker.lifetime = rclcpp::Duration(500ms);
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = latest_height_.data;

    float prob = (1 - min_prob_) * i / 4 + min_prob_;
    marker.color = util::toRgba(prob);
    marker.color.a = 0.3f;
    marker.scale.x = 0.1;
    drawCircle(marker.points, inversePdf(prob, fixed));
    array_msg.markers.push_back(marker);
  }
  marker_pub_->publish(array_msg);
}

GnssParticleCorrector::ParticleArray GnssParticleCorrector::weightParticles(
  const ParticleArray & predicted_particles, const Eigen::Vector3f & pose, bool fixed)
{
  ParticleArray weighted_particles{predicted_particles};

  for (int i{0}; i < static_cast<int>(predicted_particles.particles.size()); i++) {
    float distance{static_cast<float>(std::hypot(
      predicted_particles.particles[i].pose.position.x - pose.x(),
      predicted_particles.particles[i].pose.position.y - pose.y()))};

    if (fixed) {
      if (distance < flat_radius_)
        weighted_particles.particles[i].weight = 1.0f;
      else
        weighted_particles.particles[i].weight = normalPdf(distance - flat_radius_, 0.0, sigma_);
    } else {
      if (distance < float_range_gain_ * flat_radius_)
        weighted_particles.particles[i].weight = 1.0f;
      else
        weighted_particles.particles[i].weight =
          normalPdf(distance - float_range_gain_ * flat_radius_, 0.0, float_range_gain_ * sigma_);
    }
    weighted_particles.particles[i].weight =
      std::max(weighted_particles.particles[i].weight, min_prob_);
  }

  return weighted_particles;
}

float GnssParticleCorrector::normalPdf(float x, float mu, float sigma)
{
  // NOTE: This is not exact normal distribution
  float a = (x - mu) / sigma;
  return std::exp(-0.5f * a * a);
}

float GnssParticleCorrector::inversePdf(float prob, bool fixed) const
{
  float gain = (fixed) ? 1 : float_range_gain_;

  if (prob > 1 - 1e-6f) return gain * flat_radius_;
  if (prob < min_prob_) return gain * flat_radius_ + sigma_ * std::sqrt(-2.f * std::log(min_prob_));
  return gain * flat_radius_ + sigma_ * std::sqrt(-2.f * std::log(prob));
}

}  // namespace particle_filter