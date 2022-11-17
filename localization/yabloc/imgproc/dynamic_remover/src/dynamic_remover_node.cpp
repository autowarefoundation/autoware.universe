#include "dynamic_remover/dynamic_remover.hpp"

#include <glog/logging.h>

int main(int argc, char * argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imgproc::Reprojector>());
  rclcpp::shutdown();
  return 0;
}