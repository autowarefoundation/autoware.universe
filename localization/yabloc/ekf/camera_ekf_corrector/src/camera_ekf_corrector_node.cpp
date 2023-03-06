#include "camera_ekf_corrector/camera_ekf_corrector.hpp"

#include <glog/logging.h>

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::ekf_corrector::CameraEkfCorrector>());
  rclcpp::shutdown();
  return 0;
}