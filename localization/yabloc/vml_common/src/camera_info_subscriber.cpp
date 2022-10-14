#include "vml_common/camera_info_subscriber.hpp"

namespace vml_common
{
CameraInfoSubscriber::CameraInfoSubscriber(rclcpp::Node * node)
{
  rclcpp::QoS qos(10);
  auto callback = [this](const CameraInfo & msg) -> void { opt_info_ = msg; };
  sub_info_ = node->create_subscription<CameraInfo>("camera_info", qos, callback);
}

bool CameraInfoSubscriber::isCameraInfoReady() const { return opt_info_.has_value(); }

bool CameraInfoSubscriber::isCameraInfoNullOpt() const { return !(opt_info_.has_value()); }

std::string CameraInfoSubscriber::getFrameId() const { return opt_info_->header.frame_id; }

Eigen::Matrix3f CameraInfoSubscriber::intrinsic() const
{
  if (!opt_info_.has_value()) {
    throw std::runtime_error("camera_info is not ready but it's accessed");
  }
  const Eigen::Matrix3d Kd_t = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(opt_info_->k.data());
  return Kd_t.cast<float>().transpose();
}

}  // namespace vml_common