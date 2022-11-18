#include "twist_estimator/twist_estimator.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::twist_estimator::TwistEstimator>());
  rclcpp::shutdown();
  return 0;
}