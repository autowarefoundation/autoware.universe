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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace pcdless::velocity_converter
{
class PoseTwistFuser : public rclcpp::Node
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Odometry = nav_msgs::msg::Odometry;

  PoseTwistFuser() : Node("pose_tiwst_fuser")
  {
    using std::placeholders::_1;

    // Subscriber
    auto on_twist = [this](const TwistStamped & msg) -> void { this->latest_twist_stamped_ = msg; };
    auto on_pose = std::bind(&PoseTwistFuser::on_pose, this, _1);
    sub_twist_stamped_ = create_subscription<TwistStamped>("twist", 10, std::move(on_twist));
    sub_pose_stamped_ = create_subscription<PoseStamped>("pose", 10, std::move(on_pose));

    // Publisher
    pub_odometry_ = this->create_publisher<Odometry>("odometry", 10);
  }

private:
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_stamped_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_stamped_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  std::optional<TwistStamped> latest_twist_stamped_{std::nullopt};

  void on_pose(const PoseStamped & pose)
  {
    if (!latest_twist_stamped_.has_value()) return;

    Odometry msg;
    msg.header = pose.header;
    msg.pose.pose = pose.pose;
    msg.twist.twist = latest_twist_stamped_->twist;
    pub_odometry_->publish(msg);
  }
};

}  // namespace pcdless::velocity_converter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::velocity_converter::PoseTwistFuser>());
  rclcpp::shutdown();
  return 0;
}
