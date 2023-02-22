#include "gnss_ekf_corrector/gnss_ekf_corrector.hpp"

#include <pcdless_common/color.hpp>
#include <pcdless_common/fix2mgrs.hpp>
#include <pcdless_common/pose_conversions.hpp>
#include <pcdless_common/ublox_stamp.hpp>

namespace pcdless::ekf_corrector
{
GnssEkfCorrector::GnssEkfCorrector()
: Node("gnss_ekf_corrector"),
  ignore_less_than_float_(declare_parameter("ignore_less_than_float", true))
{
  using std::placeholders::_1;

  // Subscriber
  auto on_ublox = std::bind(&GnssEkfCorrector::on_ublox, this, _1);
  sub_ublox_ = create_subscription<NavPVT>("input/navpvt", 10, on_ublox);
  auto on_height = [this](const Float32 & height) { this->latest_height_ = height; };
  sub_height_ = create_subscription<Float32>("input/height", 10, on_height);
  auto on_pose = [this](const PoseCovStamped & pose) {
    this->current_position_ = common::pose_to_se3(pose.pose.pose).translation();
  };
  sub_pose_ = create_subscription<PoseCovStamped>("input/pose_with_covariance", 10, on_pose);

  // Publisher
  pub_pose_ = create_publisher<PoseCovStamped>("output/pose_cov_stamped", 10);
  marker_pub_ = create_publisher<MarkerArray>("gnss/range_marker", 10);
}

Eigen::Vector3f extract_enu_vel(const GnssEkfCorrector::NavPVT & msg)
{
  Eigen::Vector3f enu_vel;
  enu_vel << msg.vel_e * 1e-3, msg.vel_n * 1e-3, -msg.vel_d * 1e-3;
  return enu_vel;
}

void GnssEkfCorrector::on_ublox(const NavPVT::ConstSharedPtr ublox_msg)
{
  const rclcpp::Time stamp = common::ublox_time_to_stamp(*ublox_msg);
  const Eigen::Vector3f gnss_position = common::ublox_to_mgrs(*ublox_msg).cast<float>();

  // Check measurement certainty
  const int FIX_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FIXED;
  const int FLOAT_FLAG = ublox_msgs::msg::NavPVT::CARRIER_PHASE_FLOAT;
  const bool is_rtk_fixed = (ublox_msg->flags & FIX_FLAG);

  publish_marker(gnss_position, is_rtk_fixed);

  if (ignore_less_than_float_) {
    if (!(ublox_msg->flags & FIX_FLAG) & !(ublox_msg->flags & FLOAT_FLAG)) {
      return;
    }
  }

  PoseCovStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = stamp;
  pose.pose.pose.position.x = gnss_position.x();
  pose.pose.pose.position.y = gnss_position.y();
  pose.pose.pose.position.z = latest_height_.data;

  for (auto & e : pose.pose.covariance) e = 0;

  if (is_rtk_fixed) {
    pose.pose.covariance[6 * 0 + 0] = 1;
    pose.pose.covariance[6 * 1 + 1] = 1;
    pose.pose.covariance[6 * 2 + 2] = 1;
  } else {
    pose.pose.covariance[6 * 0 + 0] = 36;
    pose.pose.covariance[6 * 1 + 1] = 36;
    pose.pose.covariance[6 * 2 + 2] = 1;
  }

  const Eigen::Vector3f doppler = extract_enu_vel(*ublox_msg);
  const float theta = std::atan2(doppler.y(), doppler.x());

  pose.pose.pose.orientation.w = std::cos(theta / 2);
  pose.pose.pose.orientation.z = std::sin(theta / 2);
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;

  if (doppler.norm() > 1) {
    pose.pose.covariance[6 * 3 + 3] = 0.25;   // 30[deg]
    pose.pose.covariance[6 * 4 + 4] = 0.25;   // 30[deg]
    pose.pose.covariance[6 * 5 + 5] = 0.069;  // 15[deg]
  } else {
    pose.pose.covariance[6 * 3 + 3] = 0.25;  // 30[deg]
    pose.pose.covariance[6 * 4 + 4] = 0.25;  // 30[deg]
    pose.pose.covariance[6 * 5 + 5] = 1.0;   // 60[deg]
  }

  // NOTE: Removed: checking validity of GNSS measurement by mahalanobis distance

  // Compute travel distance from last update position
  // If the distance is too short, skip weighting
  {
    static Eigen::Vector3f last_position = Eigen::Vector3f::Zero();
    const float travel_distance = (current_position_ - last_position).norm();
    if (travel_distance > 1) {
      pub_pose_->publish(pose);
      last_position = current_position_;
    } else {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skip weighting because almost same positon");
    }
  }
}

void GnssEkfCorrector::publish_marker(const Eigen::Vector3f & position, bool is_rtk_fixed)
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

    float prob = i / 4.0f;
    marker.color = common::color_scale::rainbow(prob);
    marker.color.a = 0.5f;
    marker.scale.x = 0.1;
    if (is_rtk_fixed)
      drawCircle(marker.points, 1 + i);
    else
      drawCircle(marker.points, 4 + i);
    array_msg.markers.push_back(marker);
  }
  marker_pub_->publish(array_msg);
}

}  // namespace pcdless::ekf_corrector