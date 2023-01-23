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
  height_sub_ = create_subscription<Float32>("input/height", 10, on_height);

  // Publisher
  pub_pose_ = create_publisher<PoseCovStamped>("output/pose_cov_stamped", 10);
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
    pose.pose.covariance[6 * 0 + 0] = 4;
    pose.pose.covariance[6 * 1 + 1] = 4;
    pose.pose.covariance[6 * 2 + 2] = 1;
  } else {
    pose.pose.covariance[6 * 0 + 0] = 16;
    pose.pose.covariance[6 * 1 + 1] = 16;
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

  pub_pose_->publish(pose);

  // NOTE: Removed: checking validity of GNSS measurement by mahalanobis distance
  // NOTE: Removed: skiping update due to little travel distance
}

}  // namespace pcdless::ekf_corrector