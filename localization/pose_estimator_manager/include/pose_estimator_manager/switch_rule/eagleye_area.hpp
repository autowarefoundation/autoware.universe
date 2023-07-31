#pragma once

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/geometry.hpp>

#include <memory>

namespace multi_pose_estimator
{
class EagleyeArea
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  EagleyeArea();

  void init(const HADMapBin::ConstSharedPtr msg);

  bool within(const geometry_msgs::msg::Point & point) const;

  std::string debug_string() const;

  MarkerArray debug_marker_array() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace multi_pose_estimator