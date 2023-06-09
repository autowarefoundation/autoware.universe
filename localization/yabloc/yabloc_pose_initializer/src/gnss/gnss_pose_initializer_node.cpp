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

#include <rclcpp/rclcpp.hpp>
#include <yabloc_common/fix2mgrs.hpp>
#include <yabloc_common/ublox_stamp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace yabloc::particle_initializer
{
class GnssBasedPoseInitializer : public rclcpp::Node
{
public:
  using NavPVT = ublox_msgs::msg::NavPVT;
  using NavSatFix = sensor_msgs::msg::NavSatFix;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  GnssBasedPoseInitializer() : Node("gnss_based_initializer")
  {
    using std::placeholders::_1;

    // Subscriber
    auto on_navpvt = std::bind(&GnssBasedPoseInitializer::on_navpvt, this, _1);
    auto on_pf_pose = [this](const PoseStamped &) -> void { pf_is_initialized_ = true; };
    auto on_initialpose = [this](const PoseCovStamped & msg) { pub_pose_stamped_->publish(msg); };
    sub_navpvt_ = create_subscription<NavPVT>("input/ublox_topic", 10, on_navpvt);
    sub_pf_pose_ = create_subscription<PoseStamped>("input/pose", 10, on_pf_pose);
    sub_initialpose_ = create_subscription<PoseCovStamped>("input/initialpose", 10, on_initialpose);
    // Publisher
    pub_pose_stamped_ = this->create_publisher<PoseCovStamped>("output/initialpose3d", 10);
  }

private:
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pf_pose_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_stamped_;
  bool pf_is_initialized_{false};

  void on_navpvt(const NavPVT & msg)
  {
    if (pf_is_initialized_) return;

    const int FIX_FLAG = NavPVT::CARRIER_PHASE_FIXED;
    const int FLOAT_FLAG = NavPVT::CARRIER_PHASE_FLOAT;
    if (!(msg.flags & FIX_FLAG) & !(msg.flags & FLOAT_FLAG)) return;

    Eigen::Vector2f vel_xy = extract_enu_vel(msg).topRows(2);
    if (vel_xy.norm() < 5) return;
    float theta = std::atan2(vel_xy.y(), vel_xy.x());

    Eigen::Vector3f position = ublox_to_position(msg);

    PoseCovStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = common::ublox_time_to_stamp(msg);
    pose.pose.pose.position.x = position.x();
    pose.pose.pose.position.y = position.y();
    pose.pose.pose.position.z = 0;
    pose.pose.pose.orientation.w = std::cos(theta / 2.f);
    pose.pose.pose.orientation.z = std::sin(theta / 2.f);
    pub_pose_stamped_->publish(pose);
  }

  Eigen::Vector3f extract_enu_vel(const NavPVT & msg) const
  {
    Eigen::Vector3f enu_vel;
    enu_vel << msg.vel_e * 1e-3, msg.vel_n * 1e-3, -msg.vel_d * 1e-3;
    return enu_vel;
  }

  Eigen::Vector3f ublox_to_position(const NavPVT & msg)
  {
    NavSatFix fix;
    fix.latitude = msg.lat * 1e-7f;
    fix.longitude = msg.lon * 1e-7f;
    fix.altitude = msg.height * 1e-3f;
    return common::fix_to_mgrs(fix).cast<float>();
  }
};
}  // namespace yabloc::particle_initializer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<yabloc::particle_initializer::GnssBasedPoseInitializer>());
  rclcpp::shutdown();
  return 0;
}
