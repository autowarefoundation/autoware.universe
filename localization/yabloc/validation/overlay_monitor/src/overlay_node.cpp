#include "overlay_monitor/overlay_monitor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::overlay::Overlay>());
  rclcpp::shutdown();
  return 0;
}