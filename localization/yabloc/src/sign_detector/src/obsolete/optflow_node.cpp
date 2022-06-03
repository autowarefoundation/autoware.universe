#include "sign_detector/optflow.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const std::string image_topic = "/sensing/camera/traffic_light/image_raw/compressed";
  const std::string info_topic = "/sensing/camera/traffic_light/camera_info";

  rclcpp::spin(std::make_shared<OptFlowNode>(image_topic, info_topic));
  rclcpp::shutdown();
  return 0;
}