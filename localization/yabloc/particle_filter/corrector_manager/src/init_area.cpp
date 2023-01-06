#include "corrector_manager/init_area.hpp"

#include <boost/geometry/geometry.hpp>

namespace pcdless
{

InitArea::InitArea(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr points{new pcl::PointCloud<pcl::PointXYZL>()};
  pcl::fromROSMsg(msg, *points);

  if (points->empty()) return;

  polygon poly;

  std::optional<uint32_t> last_label = std::nullopt;
  for (const pcl::PointXYZL p : *points) {
    if (last_label) {
      if ((*last_label) != p.label) {
        areas_.push_back(poly);
        poly.outer().clear();
      }
    }
    poly.outer().push_back(point(p.x, p.y));
    last_label = p.label;
  }
  areas_.push_back(poly);
}

bool InitArea::is_inside(const Eigen::Vector3d & xyz) const
{
  if (areas_.empty()) return false;

  const point query(xyz.x(), xyz.y());
  for (const polygon & poly : areas_) {
    if (boost::geometry::within(query, poly)) return true;
  }
  return false;
}

}  // namespace pcdless