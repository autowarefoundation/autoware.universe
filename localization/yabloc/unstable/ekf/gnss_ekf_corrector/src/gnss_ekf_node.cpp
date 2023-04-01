#include "gnss_ekf_corrector/gnss_ekf_corrector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::ekf_corrector::GnssEkfCorrector>());
  rclcpp::shutdown();
  return 0;
}