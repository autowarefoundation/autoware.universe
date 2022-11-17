#include "gnss_particle_corrector/gnss_particle_corrector.hpp"

#include <vml_common/color.hpp>
#include <vml_common/fix2mgrs.hpp>
#include <vml_common/pose_conversions.hpp>
#include <vml_common/ublox_stamp.hpp>

namespace modularized_particle_filter
{
GnssParticleCorrector::GnssParticleCorrector()
: AbstCorrector("gnss_particle_corrector"),
  float_range_gain_(declare_parameter("float_range_gain", 5.0f)),
  likelihood_min_weight_(declare_parameter("likelihood_min_weight", 0.01f)),
  likelihood_stdev_(declare_parameter("likelihood_stdev", 5.0f)),
  likelihood_flat_radius_(declare_parameter("likelihood_flat_radius", 1.0f)),
  rtk_enabled_(declare_parameter("rtk_enabled", true))
{
  using std::placeholders::_1;

  // Subscriber
  auto cb_pose = std::bind(&GnssParticleCorrector::onPose, this, _1);
  auto cb_ublox = std::bind(&GnssParticleCorrector::onUblox, this, _1);
  auto cb_height = [this](const Float32 & height) { this->latest_height_ = height; };
  ublox_sub_ = create_subscription<NavPVT>("input/navpvt", 10, cb_ublox);
  pose_sub_ = create_subscription<PoseCovStamped>("input/pose_with_covariance", 10, cb_pose);
  height_sub_ = create_subscription<Float32>("input/height", 10, cb_height);

  // Publisher
  marker_pub_ = create_publisher<MarkerArray>("gnss/range_marker", 10);
}

void GnssParticleCorrector::onPose(const PoseCovStamped::ConstSharedPtr pose_msg)
{
  const rclcpp::Time stamp = pose_msg->header.stamp;
  std::optional<ParticleArray> opt_particles = getSynchronizedParticleArray(stamp);
  if (!opt_particles.has_value()) return;

  auto position = pose_msg->pose.pose.position;
  Eigen::Vector3f position_vec3f;
  position_vec3f << position.x, position.y, position.z;

  ParticleArray weighted_particles{weightParticles(
    opt_particles.value(), position_vec3f, likelihood_stdev_, likelihood_flat_radius_)};
  setWeightedParticleArray(weighted_particles);
}

void GnssParticleCorrector::onUblox(const NavPVT::ConstSharedPtr ublox_msg)
{
  const rclcpp::Time stamp = vml_common::ubloxTime2Stamp(*ublox_msg);

  const int FIX_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FIXED;
  const int FLOAT_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FLOAT;

  if (rtk_enabled_) {
    if (!(ublox_msg->flags & FIX_FLAG) & !(ublox_msg->flags & FLOAT_FLAG)) return;
  }

  std::optional<ParticleArray> opt_particles = getSynchronizedParticleArray(stamp);
  if (!opt_particles.has_value()) return;

  auto dt = (stamp - rclcpp::Time(opt_particles->header.stamp));
  if (std::abs(dt.seconds()) > 0.1) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Timestamp gap between gnss and particles is too large: " << dt.seconds());
  }

  NavSatFix fix;
  fix.latitude = ublox_msg->lat * 1e-7f;
  fix.longitude = ublox_msg->lon * 1e-7f;
  fix.altitude = ublox_msg->height * 1e-3f;

  const bool is_rtk_fixed = (ublox_msg->flags & FIX_FLAG);

  Eigen::Vector3f position = vml_common::fix2Mgrs(fix).cast<float>();

  float sigma = likelihood_stdev_;
  float flat_radius = likelihood_flat_radius_;
  if (!is_rtk_fixed) {
    sigma *= float_range_gain_;
    flat_radius *= float_range_gain_;
  }

  ParticleArray weighted_particles{
    weightParticles(opt_particles.value(), position, sigma, flat_radius)};

  geometry_msgs::msg::Pose mean_pose = modularized_particle_filter::meanPose(weighted_particles);
  Eigen::Vector3f mean_position = vml_common::pose2Affine(mean_pose).translation();
  if ((mean_position - last_mean_position_).squaredNorm() > 1) {
    this->setWeightedParticleArray(weighted_particles);
    last_mean_position_ = mean_position;
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *get_clock(), 2000, "Skip weighting because almost same positon");
  }

  publishMarker(position, is_rtk_fixed);
}

void GnssParticleCorrector::publishMarker(const Eigen::Vector3f & position, bool is_rtk_fixed)
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

    float prob = (1 - likelihood_min_weight_) * i / 4 + likelihood_min_weight_;
    marker.color = vml_common::color_scale::rainbow(prob);
    marker.color.a = 0.3f;
    marker.scale.x = 0.1;
    drawCircle(marker.points, inverseNormalPdf(prob, is_rtk_fixed));
    array_msg.markers.push_back(marker);
  }
  marker_pub_->publish(array_msg);
}

GnssParticleCorrector::ParticleArray GnssParticleCorrector::weightParticles(
  const ParticleArray & predicted_particles, const Eigen::Vector3f & pose, float sigma,
  float flat_radius)
{
  ParticleArray weighted_particles{predicted_particles};

  for (int i{0}; i < static_cast<int>(predicted_particles.particles.size()); i++) {
    float distance{static_cast<float>(std::hypot(
      predicted_particles.particles[i].pose.position.x - pose.x(),
      predicted_particles.particles[i].pose.position.y - pose.y()))};

    if (distance < flat_radius)
      weighted_particles.particles[i].weight = 1.0f;
    else
      weighted_particles.particles[i].weight = normalPdf(distance - flat_radius, 0.0, sigma);

    weighted_particles.particles[i].weight =
      std::max(weighted_particles.particles[i].weight, likelihood_min_weight_);
  }

  return weighted_particles;
}

float GnssParticleCorrector::normalPdf(float x, float mu, float sigma)
{
  // NOTE: This is not exact normal distribution because of no scale factor depending on sigma
  float a = (x - mu) / sigma;
  return std::exp(-0.5f * a * a);
}

float GnssParticleCorrector::inverseNormalPdf(float prob, bool is_rtk_fixed) const
{
  float gain = (is_rtk_fixed) ? 1 : float_range_gain_;

  if (prob > 1 - 1e-6f) return gain * likelihood_flat_radius_;
  if (prob < likelihood_min_weight_)
    return gain * likelihood_flat_radius_ +
           likelihood_stdev_ * std::sqrt(-2.f * std::log(likelihood_min_weight_));
  return gain * likelihood_flat_radius_ + likelihood_stdev_ * std::sqrt(-2.f * std::log(prob));
}

}  // namespace modularized_particle_filter