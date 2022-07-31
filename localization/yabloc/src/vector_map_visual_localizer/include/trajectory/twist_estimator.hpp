#pragma once
#include <eigen3/Eigen/StdVector>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
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
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using String = std_msgs::msg::String;
  using Float = std_msgs::msg::Float32;

  TwistEstimator();

private:
  enum {
    ANGLE = 0,
    VELOCITY = 1,
    BIAS = 2,
    SCALE = 3,
  };

  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;
  rclcpp::Publisher<Float>::SharedPtr pub_doppler_vel_;

  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_stamped_;

  std::optional<rclcpp::Time> last_imu_stamp_{std::nullopt};

  Eigen::Vector4f state_;  // rotation, velocity, bias, scale
  Eigen::Matrix4f cov_;
  const bool upside_down;
  const bool rtk_enabled_;

  Eigen::Matrix4f cov_predict_;
  int last_rtk_quality_{0};

  rclcpp::TimerBase::SharedPtr timer_;

  bool scale_covariance_reset_flag{false};
  void callbackTimer();

  void callbackImu(const Imu & msg);
  void callbackTwistStamped(const TwistStamped & msg);
  void callbackNavPVT(const NavPVT & msg);

  void predict();
  void measureNavPvt(const Eigen::Vector3f & v);
  void measureTwistStamped(const float vel);

  void publishTwist(const Imu & msg);
  void publishDoppler(const NavPVT & msg);
  void publishString();

  Eigen::MatrixXf rectifyPositiveSemiDefinite(const Eigen::MatrixXf & matrix);

  Eigen::Vector3f extractEnuVel(const NavPVT & msg) const;
};
}  // namespace trajectory