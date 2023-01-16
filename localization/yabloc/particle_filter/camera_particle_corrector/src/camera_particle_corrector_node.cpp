#include "camera_particle_corrector/camera_particle_corrector.hpp"

#include <glog/logging.h>

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  namespace mpf = pcdless::modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::CameraParticleCorrector>());
  rclcpp::shutdown();
  return 0;
}