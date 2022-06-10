#pragma once
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace trajectory
{
class TwistEstimator : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Imu = sensor_msgs::msg::Imu;
  using NavPVT = ublox_msgs::msg::NavPVT;

  TwistEstimator();

private:
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;

  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_stamped_;

  std::optional<rclcpp::Time> last_imu_stamp_{std::nullopt};

  Eigen::Vector4f state_;  // rotation, velocity, bias, scale
  Eigen::Matrix4f cov_;

  Eigen::Matrix4f cov_predict_;

  void callbackImu(const Imu & msg);
  void callbackTwistStamped(const TwistStamped & msg);
  void callbackNavPVT(const NavPVT & msg);

  void predict();
  void measureNavPvt(const Eigen::Vector3f & v);
  void measureTwistStamped(const float vel);

  void publishTwist(const Imu & msg);

  Eigen::Vector2f extractEnuVel(const NavPVT & msg) const;
};
}  // namespace trajectory