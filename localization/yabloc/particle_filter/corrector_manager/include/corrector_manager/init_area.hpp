#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcdless
{
class InitArea
{
public:
  using point = boost::geometry::model::d2::point_xy<double>;
  using polygon = boost::geometry::model::polygon<point>;

  InitArea(const sensor_msgs::msg::PointCloud2 & msg);

  // Return <whether it is inside or not, whether it is init area or not>
  bool is_inside(const Eigen::Vector3d & xyz, bool * init_area) const;

private:
  std::vector<polygon> init_areas_;
  std::vector<polygon> deinit_areas_;
};

}  // namespace pcdless