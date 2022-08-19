#include "vmvl_trajectory/twist_estimator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vmvl_trajectory::TwistEstimator>());
  rclcpp::shutdown();
  return 0;
}