#include "trajectory/twist_estimator.hpp"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/SVD>
#include <sophus/geometry.hpp>

namespace trajectory
{
TwistEstimator::TwistEstimator()
: Node("twist_estimaotr"),
  upside_down(true),
  rtk_enabled_(declare_parameter("rtk_enabled", true)),
  stop_vel_threshold_(declare_parameter("stop_vel_threshold", 0.05f))
{
  using std::placeholders::_1;
  using namespace std::literals::chrono_literals;

  auto cb_imu = std::bind(&TwistEstimator::callbackImu, this, _1);
  auto cb_pvt = std::bind(&TwistEstimator::callbackNavPVT, this, _1);
  auto cb_twist = std::bind(&TwistEstimator::callbackTwistStamped, this, _1);

  sub_imu_ = create_subscription<Imu>("/sensing/imu/tamagawa/imu_raw", 10, cb_imu);
  sub_twist_stamped_ = create_subscription<TwistStamped>("/vehicle/status/twist", 10, cb_twist);
  sub_navpvt_ = create_subscription<NavPVT>("/sensing/gnss/ublox/navpvt", 10, cb_pvt);

  pub_twist_ = create_publisher<TwistStamped>("/kalman/twist", 10);
  pub_pose_ = create_publisher<PoseStamped>("/kalman/doppler", 10);
  pub_string_ = create_publisher<String>("/kalman/status", 10);
  pub_doppler_vel_ = create_publisher<Float>("/kalman/doppler_vel", 10);
  pub_twist_with_covariance_ =
    create_publisher<TwistCovStamped>("/kalman/twist_with_covariance", 10);

  // rotation, velocity, bias, scale
  state_ = Eigen::Vector4f(0, 20, 0, 1);
  cov_ = Eigen::Vector4f(81, 400, 1e-6f, 1e-5f).asDiagonal();
  cov_predict_ = Eigen::Vector4f(0.01, 100, 1e-4f, 1e-4f).asDiagonal();

  auto cb_timer = std::bind(&TwistEstimator::callbackTimer, this);
  timer_ = rclcpp::create_timer(this, this->get_clock(), 3s, std::move(cb_timer));
}

void TwistEstimator::callbackTimer()
{
  if (scale_covariance_reset_flag) {
    cov_ = rectifyPositiveSemiDefinite(cov_);
    cov_(1, 3) = 0;
    cov_(3, 1) = 0;
    RCLCPP_INFO_STREAM(get_logger(), "reset covariance because unreached GNSS");
  }
  scale_covariance_reset_flag = true;
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
  state_[ANGLE] += (w_z + state_[BIAS]) * dt;

  // Update covariance
  Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
  F(0, 2) = dt;
  cov_ = F * cov_ * F.transpose() + cov_predict_ * dt * dt;

  publishTwist(msg);
  publishString();

  if (std::abs(state_[VELOCITY]) > stop_vel_threshold_) return;

  {
    // Compute error and jacobian
    float error = w_z + state_[BIAS];
    Eigen::Matrix<float, 1, 4> H;
    H << 0, 0, -1, 0;

    // Determain kalman gain
    float W = 0.001;  // [(rad/s)^2]
    float S = H * cov_ * H.transpose() + W;
    Eigen::Matrix<float, 4, 1> K = cov_ * H.transpose() / S;

    // Correct state and covariance
    state_ += K * error;
    cov_ = (Eigen::Matrix4f::Identity() - K * H) * cov_;
  }
}

void TwistEstimator::publishTwist(const Imu & imu)
{
  TwistStamped msg;
  msg.header.stamp = imu.header.stamp;
  msg.header.frame_id = "base_link";
  msg.twist.angular.z = imu.angular_velocity.z + state_[BIAS];
  msg.twist.linear.x = state_[VELOCITY];
  pub_twist_->publish(msg);

  {
    TwistCovStamped cov_msg;
    cov_msg.header = msg.header;
    cov_msg.twist.twist = msg.twist;
    cov_msg.twist.covariance.at(0) = 0.1;
    cov_msg.twist.covariance.at(35) = 0.1;
    pub_twist_with_covariance_->publish(cov_msg);
  }
}

void TwistEstimator::callbackTwistStamped(const TwistStamped & msg)
{
  static bool first_subscirbe = true;
  if (first_subscirbe) {
    float wheel = msg.twist.linear.x;
    state_[VELOCITY] = wheel;
    RCLCPP_INFO_STREAM(get_logger(), "first twist subscription: " << state_[VELOCITY]);
    first_subscirbe = false;
    return;
  }
  const float wheel = msg.twist.linear.x;

  if (std::abs(wheel) < stop_vel_threshold_) return;

  // Compute error and jacobian
  float error = state_[VELOCITY] - state_[SCALE] * wheel;
  Eigen::Matrix<float, 1, 4> H;
  H << 0, -1, 0, wheel;

  // Determain kalman gain
  float W = 0.09;  // [(m/s)^2]
  float S = H * cov_ * H.transpose() + W;
  Eigen::Matrix<float, 4, 1> K = cov_ * H.transpose() / S;

  // Correct state and covariance
  state_ += K * error;
  cov_ = (Eigen::Matrix4f::Identity() - K * H) * cov_;
}

void TwistEstimator::callbackNavPVT(const NavPVT & msg)
{
  switch (msg.flags) {
    case 131:
      last_rtk_quality_ = 2;
      break;
    case 67:
      last_rtk_quality_ = 1;
      break;
    default:
      last_rtk_quality_ = 0;
      break;
  }

  publishDoppler(msg);

  if (rtk_enabled_) {
    if ((msg.flags != 131) && (msg.flags != 67)) {
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 2000, "GNSS is unreliable!");
      return;
    }
  }

  static bool first_subscirbe = true;
  if (first_subscirbe) {
    Eigen::Vector2f vel_xy = extractEnuVel(msg).topRows(2);
    state_[ANGLE] = std::atan2(vel_xy.y(), vel_xy.x());
    RCLCPP_INFO_STREAM(get_logger(), "first navpvt subscription: " << state_[ANGLE]);
    first_subscirbe = false;
    return;
  }

  // Compute error and jacobian
  Eigen::Vector2f vel_xy = extractEnuVel(msg).topRows(2);
  Eigen::Matrix2f R = Eigen::Rotation2D(state_[ANGLE]).toRotationMatrix();
  Eigen::Vector2f error = R * state_[VELOCITY] * Eigen::Vector2f::UnitX() - vel_xy;

  float std_vel = std::sqrt(cov_(VELOCITY, VELOCITY));

  RCLCPP_INFO_STREAM(
    get_logger(),
    state_[VELOCITY] << " " << vel_xy.norm() << " " << error.transpose() << " " << std_vel);

  if (error.norm() > std_vel) {
    RCLCPP_WARN_STREAM(
      get_logger(), "skip GNSS update because velocity error is too large " << error.transpose()
                                                                            << " " << std_vel);
    return;
  }

  cov_ = rectifyPositiveSemiDefinite(cov_);

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

  Float float_msg;
  float_msg.data = vel_xy.norm();
  pub_doppler_vel_->publish(float_msg);

  scale_covariance_reset_flag = false;
}

Eigen::Vector3f TwistEstimator::extractEnuVel(const NavPVT & msg) const
{
  Eigen::Vector3f enu_vel;
  enu_vel << msg.vel_e * 1e-3, msg.vel_n * 1e-3, -msg.vel_d * 1e-3;
  return enu_vel;
}

void TwistEstimator::publishDoppler(const NavPVT & navpvt)
{
  if (!last_imu_stamp_.has_value()) return;

  PoseStamped msg;
  msg.header.stamp = last_imu_stamp_.value();
  msg.header.frame_id = "base_link";

  Eigen::Vector2f vel = extractEnuVel(navpvt).topRows(2);
  Eigen::Matrix2f R = Eigen::Rotation2D(state_[ANGLE]).toRotationMatrix();
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
  switch (last_rtk_quality_) {
    case 2:
      ss << "RTK: FIX" << std::endl;
      break;
    case 1:
      ss << "RTK: FLOAT" << std::endl;
      break;
    default:
      ss << "RTK: UNRELIABLE" << std::endl;
  }

  String msg;
  msg.data = ss.str();
  pub_string_->publish(msg);
}

Eigen::MatrixXf TwistEstimator::rectifyPositiveSemiDefinite(const Eigen::MatrixXf & matrix)
{
  Eigen::JacobiSVD<Eigen::MatrixXf> svd;
  svd.compute(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXf U = svd.matrixU();
  Eigen::VectorXf singular = svd.singularValues();
  singular = singular.cwiseMax(1e-6f);
  Eigen::MatrixXf S = singular.asDiagonal();
  return U * S * U.transpose();
}

}  // namespace trajectory