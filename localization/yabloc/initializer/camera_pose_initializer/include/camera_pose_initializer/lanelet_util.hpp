#pragma once
#include <lanelet2_core/LaneletMap.h>

#include <optional>

// TODO: Reuse autoware_common/lanelet2_extension instead of this
namespace lanelet
{
std::optional<double> get_current_direction(
  const ConstLanelets & lanelets, const Eigen::Vector3f & query_position);

bool get_current_lanelets(
  const ConstLanelets & lanelets, const Eigen::Vector3f & query_position,
  ConstLanelets * current_lanelets_ptr);

ConstLineString3d get_closest_segment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring);

}  // namespace lanelet