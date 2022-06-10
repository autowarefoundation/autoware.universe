#include "trajectory/twist_estimator.hpp"

namespace trajectory
{
TwistEstimator::TwistEstimator() : Node("twist_estimaotr")
{
  using std::placeholders::_1;

  auto cb_imu = std::bind(&TwistEstimator::callbackImu, this, _1);
  auto cb_fix = std::bind(&TwistEstimator::callbackNavSatFix, this, _1);
  auto cb_pvt = std::bind(&TwistEstimator::callbackNavPVT, this, _1);
  auto cb_twist = std::bind(&TwistEstimator::callbackTwistStamped, this, _1);

  sub_imu_ = create_subscription<Imu>("/sensing/imu/tamagawa/imu_raw", 10, cb_imu);
  sub_twist_stamped_ = create_subscription<TwistStamped>("/vehicle/status/twist", 10, cb_twist);
  sub_navpvt_ = create_subscription<NavPVT>("/sensing/gnss/ublox/navpvt", 10, cb_pvt);
  sub_fix_ = create_subscription<NavSatFix>("/sensing/gnss/ublox/nav_sat_fix", 10, cb_fix);

  pub_twist_ = create_publisher<TwistStamped>("/kalman/twist", 10);
}

void TwistEstimator::callbackImu(const Imu & msg) {}
void TwistEstimator::callbackTwistStamped(const TwistStamped & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "twist_stamped callback");
}
void TwistEstimator::callbackNavPVT(const NavPVT & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "navpvt callback");
}
void TwistEstimator::callbackNavSatFix(const NavSatFix & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "navsatfix callback");
}
}  // namespace trajectory