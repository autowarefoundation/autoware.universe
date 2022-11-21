#include "twist_estimator/twist_visualizer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::twist_visualizer::TwistVisualizer>());
  rclcpp::shutdown();
  return 0;
}