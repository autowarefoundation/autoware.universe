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
  sigma_(declare_parameter("sigma", 25.0f))
{
  using std::placeholders::_1;
  auto ublox_callback = std::bind(&GnssParticleCorrector::ubloxCallback, this, _1);
  ublox_sub_ = create_subscription<NavPVT>("/sensing/gnss/ublox/navpvt", 10, ublox_callback);

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  marker_pub_ = create_publisher<MarkerArray>("/gnss/effect_marker", 10);
}

#include <time.h>
rclcpp::Time GnssParticleCorrector::ubloxTime2Stamp(const NavPVT & msg)
{
  struct tm t;
  t.tm_year = msg.year - 1900;  // from 1900
  t.tm_mon = msg.month - 1;     // january = 0
  t.tm_mday = msg.day;
  t.tm_hour = msg.hour + 9;
  t.tm_min = msg.min;
  t.tm_sec = msg.sec;
  t.tm_isdst = 0;
  time_t t_of_day = mktime(&t);

  rclcpp::Time stamp(t_of_day, msg.nano, RCL_ROS_TIME);
  return stamp;
}

void GnssParticleCorrector::ubloxCallback(const NavPVT::ConstSharedPtr ublox_msg)
{
  const rclcpp::Time stamp = ubloxTime2Stamp(*ublox_msg);

  const int FIX_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FIXED;
  const int FLOAT_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FLOAT;

  if (!(ublox_msg->flags & FIX_FLAG) & !(ublox_msg->flags & FLOAT_FLAG)) return;

  std::optional<ParticleArray> opt_particles = getSyncronizedParticleArray(stamp);
  if (!opt_particles.has_value()) {
    RCLCPP_WARN_STREAM(get_logger(), "ubloxCallback does not have opt_particles");
    return;
  }

  auto dt = (stamp - rclcpp::Time(opt_particles->header.stamp));
  RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt.seconds());

  NavSatFix fix;
  fix.latitude = ublox_msg->lat * 1e-7f;
  fix.longitude = ublox_msg->lon * 1e-7f;
  fix.altitude = ublox_msg->height * 1e-3f;

  Eigen::Vector3f position = fix2Mgrs(fix).cast<float>();
  ParticleArray weighted_particles{weightParticles(opt_particles.value(), position)};
  setWeightedParticleArray(weighted_particles);

  publishMarker(position);
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

  publishMarker(position);
}

void GnssParticleCorrector::publishMarker(const Eigen::Vector3f & position)
{
  MarkerArray array_msg;
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

  for (int i = 0; i < 5; i++) {
    Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = "/map";
    marker.id = i;
    marker.type = Marker::LINE_STRIP;
    marker.lifetime = rclcpp::Duration(500ms);
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();

    float prob = (1 - min_prob_) * i / 4 + min_prob_;
    marker.color = util::toRgba(prob);
    marker.color.a = 0.2f;
    marker.scale.x = 0.1;
    drawCircle(marker.points, inversePdf(prob));
    array_msg.markers.push_back(marker);
  }
  marker_pub_->publish(array_msg);
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

float GnssParticleCorrector::inversePdf(float prob) const
{
  if (prob > 1 - 1e-6f) return flat_radius_;
  if (prob < min_prob_) return flat_radius_ + sigma_ * std::sqrt(-2.f * std::log(min_prob_));
  return flat_radius_ + sigma_ * std::sqrt(-2.f * std::log(prob));
}

}  // namespace particle_filter