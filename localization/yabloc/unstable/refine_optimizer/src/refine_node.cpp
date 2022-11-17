#include "refine_optimizer/refine.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<refine_optimizer::RefineOptimizer>());
  rclcpp::shutdown();
  return 0;
}