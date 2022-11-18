#include "antishadow_corrector/antishadow_corrector.hpp"

int main(int argc, char * argv[])
{
  namespace mpf = pcdless::modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::AntishadowCorrector>());
  rclcpp::shutdown();
  return 0;
}
