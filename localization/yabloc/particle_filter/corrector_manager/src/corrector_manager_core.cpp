#include "corrector_manager/corrector_manager.hpp"

namespace pcdless
{
CorrectorManager::CorrectorManager() : Node("predictor")
{
  using std::placeholders::_1;
  auto on_gnss_pose = std::bind(&CorrectorManager::on_gnss_pose, this, _1);
  auto on_init_area = std::bind(&CorrectorManager::on_init_area, this, _1);
  sub_init_area_ =
    create_subscription<PointCloud2>("/localization/map/ll2_polygon", 10, on_init_area);
  sub_gnss_pose_ = create_subscription<PoseStamped>("/sensing/gnss/ublox/pose", 10, on_gnss_pose);
}

void CorrectorManager::on_init_area(const PointCloud2 & msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "initialize pcdless init areas");
  init_area_ = InitArea(msg);
}

void CorrectorManager::on_gnss_pose(const PoseStamped & msg)
{
  if (init_area_) {
    auto p = msg.pose.position;
    if (init_area_->is_inside({p.x, p.y, p.z})) {
      RCLCPP_WARN_STREAM(get_logger(), "Initialize pose because gnss enters initializable area");
      // TODO:
    }
  }
}

}  // namespace pcdless