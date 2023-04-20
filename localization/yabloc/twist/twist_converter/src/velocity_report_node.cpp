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

class VelocityReportEncoder : public rclcpp::Node
{
public:
  using Velocity = autoware_auto_vehicle_msgs::msg::VelocityReport;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  VelocityReportEncoder() : Node("vehicle_report")
  {
    using std::placeholders::_1;

    // Subscriber
    auto cb_twist = std::bind(&VelocityReportEncoder::on_twist, this, _1);
    sub_twist_ =
      create_subscription<TwistStamped>("/vehicle/status/twist", 10, cb_twist);

    // Publisher
    pub_velocity_report_ = create_publisher<Velocity>("/vehicle/status/velocity_status", 10);
  }

private:
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Publisher<Velocity>::SharedPtr pub_velocity_report_;

  void on_twist(const TwistStamped & msg)
  {
    VelocityReport velocity_report;
    velocity_report.header = msg.header;
    velocity_report.longitudinal_velocity = msg.twist.linear.x;
    velocity_report.lateral_velocity = msg.twist.linear.y;
    velocity_report.heading_rate = msg.heading_rate;
    pub_twist_stamped_->publish(velocity_report);
  }
};

}  // namespace pcdless::velocity_conveter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::velocity_conveter::VelocityReportEncoder>());
  rclcpp::shutdown();
  return 0;
}