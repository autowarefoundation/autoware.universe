#include "camera_pose_initializer/lanelet_util.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

namespace lanelet
{

bool get_current_lanelets(
  const ConstLanelets & lanelets, const Eigen::Vector3f & query_position,
  ConstLanelets * current_lanelets_ptr)
{
  if (current_lanelets_ptr == nullptr) {
    std::cerr << "argument closest_lanelet_ptr is null! Failed to find closest lanelet"
              << std::endl;
    return false;
  }

  if (lanelets.empty()) {
    return false;
  }

  lanelet::BasicPoint2d search_point(query_position.x(), query_position.y());
  for (const auto & llt : lanelets) {
    if (lanelet::geometry::inside(llt, search_point)) {
      current_lanelets_ptr->push_back(llt);
    }
  }

  return !current_lanelets_ptr->empty();  // return found
}

ConstLineString3d get_closest_segment(
  const lanelet::BasicPoint2d & search_pt, const lanelet::ConstLineString3d & linestring)
{
  if (linestring.size() < 2) {
    return lanelet::LineString3d();
  }

  lanelet::ConstLineString3d closest_segment;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 1; i < linestring.size(); i++) {
    lanelet::BasicPoint3d prev_basic_pt = linestring[i - 1].basicPoint();
    lanelet::BasicPoint3d current_basic_pt = linestring[i].basicPoint();

    lanelet::Point3d prev_pt(
      lanelet::InvalId, prev_basic_pt.x(), prev_basic_pt.y(), prev_basic_pt.z());
    lanelet::Point3d current_pt(
      lanelet::InvalId, current_basic_pt.x(), current_basic_pt.y(), current_basic_pt.z());

    lanelet::LineString3d current_segment(lanelet::InvalId, {prev_pt, current_pt});
    double distance = lanelet::geometry::distance2d(
      lanelet::utils::to2D(current_segment).basicLineString(), search_pt);
    if (distance < min_distance) {
      closest_segment = current_segment;
      min_distance = distance;
    }
  }
  return closest_segment;
}

std::optional<double> get_current_direction(
  const ConstLanelets & lanelets, const Eigen::Vector3f & query_position)
{
  ConstLanelets current_lanelets;
  bool found = get_current_lanelets(lanelets, query_position, &current_lanelets);

  lanelet::BasicPoint2d search_pt(query_position.x(), query_position.y());

  if (found) {
    auto lanelet = current_lanelets.front();  // TODO: consider all lanelet too
    lanelet::ConstLineString3d segment = get_closest_segment(search_pt, lanelet.centerline());
    return std::atan2(
      segment.back().y() - segment.front().y(), segment.back().x() - segment.front().x());
  } else {
    return std::nullopt;
  }
}

}  // namespace lanelet
