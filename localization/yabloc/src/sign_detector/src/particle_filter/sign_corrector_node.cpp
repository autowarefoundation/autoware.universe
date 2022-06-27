#include "particle_filter/sign_corrector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<particle_filter::SignCorrector>());
  rclcpp::shutdown();
  return 0;
}