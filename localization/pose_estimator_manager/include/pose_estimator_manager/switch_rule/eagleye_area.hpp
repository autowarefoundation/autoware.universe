#pragma once

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/geometry/geometry.hpp>

#include <memory>

namespace multi_pose_estimator
{
class EagleyeArea
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;

  EagleyeArea();

  void init(HADMapBin::ConstSharedPtr msg);

  bool within(geometry_msgs::msg::Point & point);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace multi_pose_estimator