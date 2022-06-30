#include "imgproc/vanish.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<impgroc::VanishPoint>());
  rclcpp::shutdown();
  return 0;
}
