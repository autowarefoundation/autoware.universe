#include "pcdless_common/camera_info_subscriber.hpp"

namespace pcdless::common
{
CameraInfoSubscriber::CameraInfoSubscriber(rclcpp::Node * node)
{
  rclcpp::QoS qos(10);
  auto callback = [this](const CameraInfo & msg) -> void { opt_info_ = msg; };
  sub_info_ = node->create_subscription<CameraInfo>("camera_info", qos, callback);
}

bool CameraInfoSubscriber::is_camera_info_ready() const { return opt_info_.has_value(); }

bool CameraInfoSubscriber::is_camera_info_nullopt() const { return !(opt_info_.has_value()); }

std::string CameraInfoSubscriber::get_frame_id() const { return opt_info_->header.frame_id; }

Eigen::Matrix3f CameraInfoSubscriber::intrinsic() const
{
  if (!opt_info_.has_value()) {
    throw std::runtime_error("camera_info is not ready but it's accessed");
  }
  const Eigen::Matrix3d Kd_t = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(opt_info_->k.data());
  return Kd_t.cast<float>().transpose();
}

}  // namespace pcdless::common