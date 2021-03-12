// Copyright 2021 Tier IV, Inc.
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

#include <cstdlib>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "autoware_system_msgs/msg/autoware_version.hpp"

class AutowareVersionNode : public rclcpp::Node
{
public:
  AutowareVersionNode()
  : Node("autoware_version_node")
  {
    autoware_system_msgs::msg::AutowareVersion message;
    {
      const char * ptr = std::getenv("ROS_VERSION");
      if (ptr) {
        message.ros_version = std::atoi(ptr);
      }
    }
    RCLCPP_INFO(get_logger(), "ROS_VERSION=%d", message.ros_version);
    {
      const char * ptr = std::getenv("ROS_DISTRO");
      if (ptr) {
        message.ros_distro = ptr;
      }
    }
    RCLCPP_INFO(get_logger(), "ROS_DISTRO=%s", message.ros_distro.c_str());

    publisher_ = create_publisher<autoware_system_msgs::msg::AutowareVersion>(
      "autoware_version", rclcpp::QoS{1}.transient_local());
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<autoware_system_msgs::msg::AutowareVersion>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareVersionNode>());
  rclcpp::shutdown();
  return 0;
}
