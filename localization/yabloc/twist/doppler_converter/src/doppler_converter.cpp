#include <pcdless_common/ublox_stamp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <septentrio_gnss_driver_msgs/msg/pvt_geodetic.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace pcdless::doppler_conveter
{
class DopplerConverter : public rclcpp::Node
{
public:
  using PVTGeodetic = septentrio_gnss_driver_msgs::msg::PVTGeodetic;
  using NavPVT = ublox_msgs::msg::NavPVT;

  DopplerConverter() : Node("doppler_converter")
  {
    using std::placeholders::_1;

    // Subscriber
    auto cb_septentrio = std::bind(&DopplerConverter::on_pvtgeodetic, this, _1);
    sub_pvtgeodetic_ =
      create_subscription<PVTGeodetic>("/sensing/gnss/septentrio/pvtgeodetic", 10, cb_septentrio);

    // Publisher
    pub_navpvt_ = create_publisher<NavPVT>("/sensing/gnss/ublox/navpvt", 10);
  }

private:
  rclcpp::Subscription<PVTGeodetic>::SharedPtr sub_pvtgeodetic_;
  rclcpp::Publisher<NavPVT>::SharedPtr pub_navpvt_;

  void on_pvtgeodetic(const PVTGeodetic & src)
  {
    auto rad_to_deg = [](double rad) -> double { return rad * 180 / M_PI; };
    if (src.mode == 0) {
      return;
    }

    NavPVT dst = common::stamp_to_ublox_time(src.header.stamp);
    dst.vel_n = src.vn;
    dst.vel_e = src.ve;
    dst.vel_d = -src.vu;
    dst.lat = rad_to_deg(src.latitude) * 1e7;
    dst.lon = rad_to_deg(src.longitude) * 1e7;

    // NOTE:
    // https://github.com/tier4/septentrio_gnss_driver/blob/tier4/ros2/septentrio_gnss_driver/src/septentrio_gnss_driver/communication/rx_message.cpp#L53-L65
    // https://github.com/KumarRobotics/ublox/blob/4f107f3b82135160a1aca3ef0689fd119199bbef/ublox_msgs/msg/NavPVT.msg#L45-L62
    if (src.mode == 4) {
      dst.flags = NavPVT::FLAGS_GNSS_FIX_OK + NavPVT::CARRIER_PHASE_FIXED;
    } else if (src.mode == 5) {
      dst.flags = NavPVT::FLAGS_GNSS_FIX_OK + NavPVT::CARRIER_PHASE_FLOAT;
    } else {
      dst.flags = NavPVT::FLAGS_GNSS_FIX_OK;
    }

    dst.height = src.height;
    pub_navpvt_->publish(dst);
  }
};
}  // namespace pcdless::doppler_conveter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pcdless::doppler_conveter::DopplerConverter>());
  rclcpp::shutdown();
  return 0;
}
