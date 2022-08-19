#include "vml_common/base_camera_info_node.hpp"

namespace vml_common
{
BaseCameraInfoNode::BaseCameraInfoNode(
  const std::string node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  auto cb_info = [this](const CameraInfo & msg) -> void { opt_info_ = msg; };
  sub_info_ = create_subscription<CameraInfo>("/input/camera_info", 10, cb_info);
}

bool BaseCameraInfoNode::isCameraInfoReady() const { return opt_info_.has_value(); }

bool BaseCameraInfoNode::isCameraInfoNullOpt() const { return (!opt_info_.has_value()); }

Eigen::Matrix3f BaseCameraInfoNode::intrinsic() const
{
  if (!opt_info_.has_value()) {
    throw std::runtime_error("camera_info is not ready but it's accessed");
  }
  const Eigen::Matrix3d Kd_t = Eigen::Map<const Eigen::Matrix<double, 3, 3>>(opt_info_->k.data());
  return Kd_t.cast<float>().transpose();
}
}  // namespace vml_common