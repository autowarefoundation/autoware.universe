#include "trajectory/twist_estimator.hpp"

#include <eigen3/Eigen/Geometry>
#include <sophus/geometry.hpp>

namespace trajectory
{
TwistEstimator::TwistEstimator() : Node("twist_estimaotr"), upside_down(true)
{
  using std::placeholders::_1;

  auto cb_imu = std::bind(&TwistEstimator::callbackImu, this, _1);
  auto cb_pvt = std::bind(&TwistEstimator::callbackNavPVT, this, _1);
  auto cb_twist = std::bind(&TwistEstimator::callbackTwistStamped, this, _1);

  sub_imu_ = create_subscription<Imu>("/sensing/imu/tamagawa/imu_raw", 10, cb_imu);
  sub_twist_stamped_ = create_subscription<TwistStamped>("/vehicle/status/twist", 10, cb_twist);
  sub_navpvt_ = create_subscription<NavPVT>("/sensing/gnss/ublox/navpvt", 10, cb_pvt);

  pub_twist_ = create_publisher<TwistStamped>("/kalman/twist", 10);
  pub_pose_ = create_publisher<PoseStamped>("/kalman/doppler", 10);
  pub_string_ = create_publisher<String>("/kalman/status", 10);

  // rotation, velocity, bias, scale
  state_ = Eigen::Vector4f(0, 0, 0, 1);
  cov_ = Eigen::Vector4f(81, 400, 1e-7f, 0.00001).asDiagonal();
  cov_predict_ = Eigen::Vector4f(0.01, 100, 0.0001, 0.00001).asDiagonal();
}

void TwistEstimator::callbackImu(const Imu & raw_msg)
{
  if (!last_imu_stamp_.has_value()) {
    last_imu_stamp_ = raw_msg.header.stamp;
    return;
  }
  Imu msg = raw_msg;
  if (upside_down) {
    msg.angular_velocity.z = -msg.angular_velocity.z;
  }

  auto dt = (rclcpp::Time(msg.header.stamp) - last_imu_stamp_.value()).seconds();
  last_imu_stamp_ = msg.header.stamp;

  // Update state
  float w_z = msg.angular_velocity.z;
  state_[0] += (w_z + state_[2]) * dt;

  // Update covariance
  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = dt;
  cov_ = F * cov_ * F.transpose() + cov_predict_ * dt * dt;

  publishTwist(msg);
  publishString();
}

void TwistEstimator::publishTwist(const Imu & imu)
{
  TwistStamped msg;
  msg.header.stamp = imu.header.stamp;
  msg.header.frame_id = "base_link";
  msg.twist.angular.z = imu.angular_velocity.z + state_[2];
  msg.twist.linear.x = state_[1];
  pub_twist_->publish(msg);
}

void TwistEstimator::callbackTwistStamped(const TwistStamped & msg)
{
  // Compute error and jacobian
  float wheel = msg.twist.linear.x;
  float error = state_[1] - state_[3] * wheel;
  Eigen::Matrix<float, 1, 4> H;
  H << 0, -1, 0, wheel;

  // Determain kalman gain
  float W = 0.09;  // [(m/s)^2]
  float S = H * cov_ * H.transpose() + W;
  Eigen::Matrix<float, 4, 1> K = cov_ * H.transpose() / S;

  // Correct state and covariance
  state_ += K * error;
  cov_ = (Eigen::Matrix4f::Identity() - K * H) * cov_;

  RCLCPP_INFO_STREAM(get_logger(), state_[1] << " " << state_[3] * wheel << " " << wheel);
}

void TwistEstimator::callbackNavPVT(const NavPVT & msg)
{
  last_rtk_fixed_ = (msg.flags == 131);
  if ((msg.flags != 131) && (msg.flags != 67)) {
    RCLCPP_WARN(get_logger(), "NOT FIX!");
    return;
  }

  // Compute error and jacobian
  Eigen::Vector2f vel_xy = extractEnuVel(msg).topRows(2);
  Eigen::Matrix2f R = Eigen::Rotation2D(state_[0]).toRotationMatrix();
  Eigen::Vector2f error = R * state_[1] * Eigen::Vector2f::UnitX() - vel_xy;

  // Determain kalman gain
  Eigen::Matrix2f dR;
  dR << 0, -1, 1, 0;
  Eigen::Matrix<float, 2, 4> H;
  H.setZero();
  H.block<2, 1>(0, 0) = -R * dR * state_[1] * Eigen::Vector2f::UnitX();
  H.block<2, 1>(0, 1) = -R * Eigen::Vector2f::UnitX();
  Eigen::Matrix2f W = Eigen::Vector2f(1, 1).asDiagonal();
  Eigen::Matrix2f S = H * cov_ * H.transpose() + W;
  Eigen::Matrix<float, 4, 2> K = cov_ * H.transpose() * S.inverse();

  // Correct state and covariance
  state_ += K * error;
  cov_ = (Eigen::Matrix4f::Identity() - K * H) * cov_;

  publishDoppler(msg);
}

Eigen::Vector3f TwistEstimator::extractEnuVel(const NavPVT & msg) const
{
  Eigen::Vector3f enu_vel;
  enu_vel << msg.vel_e * 1e-3, msg.vel_n * 1e-3, -msg.vel_d * 1e-3;
  return enu_vel;
}

void TwistEstimator::publishDoppler(const NavPVT & navpvt)
{
  PoseStamped msg;
  msg.header.stamp = last_imu_stamp_.value();
  msg.header.frame_id = "base_link";

  Eigen::Vector2f vel = extractEnuVel(navpvt).topRows(2);
  Eigen::Matrix2f R = Eigen::Rotation2D(state_[0]).toRotationMatrix();
  vel = R.transpose() * vel;

  float theta = std::atan2(vel.y(), vel.x());
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = std::sin(theta / 2.0f);
  msg.pose.orientation.w = std::cos(theta / 2.0f);
  pub_pose_->publish(msg);
}

void TwistEstimator::publishString()
{
  std::stringstream ss;
  ss << "--- Twist Estimator Status ----" << std::endl;
  ss << "angle: " << state_[0] << std::endl;
  ss << "vel: " << state_[1] << std::endl;
  ss << "bias: " << state_[2] << std::endl;
  ss << "scale: " << state_[3] << std::endl;
  ss << std::endl;
  if (last_rtk_fixed_)
    ss << "RTK: FIX" << std::endl;
  else
    ss << "RTK: !!NOT FIX!!" << std::endl;

  String msg;
  msg.data = ss.str();
  pub_string_->publish(msg);
}

}  // namespace trajectory