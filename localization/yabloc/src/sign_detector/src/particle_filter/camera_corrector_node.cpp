#include "particle_filter/camera_corrector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<particle_filter::CameraParticleCorrector>());
  rclcpp::shutdown();
  return 0;
}