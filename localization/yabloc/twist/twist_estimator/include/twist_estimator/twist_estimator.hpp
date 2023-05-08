// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

#include <optional>

namespace yabloc::twist_estimator
{
class TwistEstimator : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using TwistCovStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;
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

  rclcpp::Publisher<PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;
  rclcpp::Publisher<Float>::SharedPtr pub_doppler_vel_;
  rclcpp::Publisher<TwistCovStamped>::SharedPtr pub_twist_cov_;

  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_report_;

  std::optional<rclcpp::Time> last_imu_stamp_{std::nullopt};

  Eigen::Vector4f state_;  // rotation, velocity, bias, scale
  Eigen::Matrix4f cov_;
  const bool upside_down;
  const bool ignore_less_than_float_;
  const float stop_vel_threshold_;
  const float static_scale_factor_;
  const float static_gyro_bias_;

  Eigen::Matrix4f cov_predict_;
  int last_rtk_quality_{0};

  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector2f last_doppler_vel_;
  float last_wheel_vel_;

  bool scale_covariance_reset_flag{false};
  void on_timer();

  void on_imu(const Imu & msg);
  void on_velocity_report(const VelocityReport & msg);
  void on_navpvt(const NavPVT & msg);

  void predict();
  void measure_navpvt(const Eigen::Vector3f & v);
  void measure_twist_stamped(const float vel);

  void publish_twist(const Imu & msg);
  void publish_doppler(const NavPVT & msg);
  void publish_string();

  Eigen::MatrixXf rectify_positive_semi_definite(const Eigen::MatrixXf & matrix);

  Eigen::Vector3f extract_enu_vel(const NavPVT & msg) const;
};
}  // namespace yabloc::twist_estimator