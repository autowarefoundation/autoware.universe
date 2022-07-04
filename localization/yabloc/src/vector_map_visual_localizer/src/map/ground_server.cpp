#include "rclcpp/rclcpp.hpp"

#include "vmvl_msgs/srv/ground.hpp"

#include <memory>

void add(
  const std::shared_ptr<vmvl_msgs::srv::Ground::Request> request,
  std::shared_ptr<vmvl_msgs::srv::Ground::Response> response)
{
  response->pose.position.x = 0;
  response->pose.position.y = 1;
  response->pose.position.z = 2;
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("rclcpp"),
    "ground service " << request->point.x << " " << request->point.y << " " << request->point.z);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ground_server");

  rclcpp::Service<vmvl_msgs::srv::Ground>::SharedPtr service =
    node->create_service<vmvl_msgs::srv::Ground>("ground", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready ground server");

  rclcpp::spin(node);
  rclcpp::shutdown();
}