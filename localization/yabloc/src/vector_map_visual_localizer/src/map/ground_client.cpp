#include "rclcpp/rclcpp.hpp"

#include "vmvl_msgs/srv/ground.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ground_client");
  rclcpp::Client<vmvl_msgs::srv::Ground>::SharedPtr client =
    node->create_client<vmvl_msgs::srv::Ground>("ground");

  auto request = std::make_shared<vmvl_msgs::srv::Ground::Request>();
  request->point.x = 3;
  request->point.y = 4;
  request->point.z = 5;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("rclcpp"), result.get()->pose.position.x
                                      << " " << result.get()->pose.position.y << " "
                                      << result.get()->pose.position.z);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ground");
  }

  rclcpp::shutdown();
  return 0;
}