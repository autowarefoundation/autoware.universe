#include "camera_particle_corrector/camera_particle_corrector.hpp"

int main(int argc, char * argv[])
{
  namespace mpf = modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::CameraParticleCorrector>());
  rclcpp::shutdown();
  return 0;
}