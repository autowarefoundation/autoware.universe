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

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace pcdless::velocity_conveter
{
class VelocityReportDecoder : public rclcpp::Node
{
public:
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Velocity = autoware_auto_vehicle_msgs::msg::VelocityReport;

  VelocityReportDecoder() : Node("vehicle_report")
  {
    using std::placeholders::_1;

    // Subscriber
    auto cb_velocity = std::bind(&VelocityReportDecoder::on_velocity, this, _1);
    sub_velocity_ =
      create_subscription<Velocity>("/vehicle/status/velocity_status", 10, cb_velocity);

    // Publisher
    pub_twist_stamped_ = create_publisher<TwistStamped>("/vehicle/status/twist", 10);
  }

private:
  rclcpp::Subscription<Velocity>::SharedPtr sub_velocity_;
  rclcpp::Publisher<TwistStamped>::SharedPtr pub_twist_stamped_;

  void on_velocity(const Velocity & msg)
  {
    TwistStamped ts;
    ts.header = msg.header;
    ts.twist.linear.x = msg.longitudinal_velocity;
    ts.twist.linear.y = msg.lateral_velocity;
    ts.twist.angular.z = msg.heading_rate;
    pub_twist_stamped_->publish(ts);
  }
};
}  // namespace pcdless::velocity_conveter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::velocity_conveter::VelocityReportDecoder>());
  rclcpp::shutdown();
  return 0;
}