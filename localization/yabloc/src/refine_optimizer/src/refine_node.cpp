#include "refine.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<validation::RefineOptimizer>());
  rclcpp::shutdown();
  return 0;
}