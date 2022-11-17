#include "ground_server/ground_server.hpp"

#include <glog/logging.h>

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map::GroundServer>());
  rclcpp::shutdown();
}