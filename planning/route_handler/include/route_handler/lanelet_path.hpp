// Copyright 2022 Macnica, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROUTE_HANDLER__LANELET_PATH_HPP_
#define ROUTE_HANDLER__LANELET_PATH_HPP_

#include "route_handler/forward.hpp"
#include "route_handler/lanelet_section.hpp"
#include <route_handler/forward.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <lanelet2_routing/Types.h>

#include <cstddef>

namespace route_handler
{

//! @brief A lanelet path with specified start and end points.
//! The path is composed of a sequence of lanelets, either directly connected to each other (the
//! next lanelet in the sequence follows the previous one), or indirectly connected through a lane
//! change (the next lanelet follows a reachable neighbor of the previous one). 
//! The start and end points refer respectively to points on the centerline of the first and last lanelets in the path. 
//! Loops are not supported within the path: any path point must only appear once within the path.
//! Operations on LaneletPath have no access to the route graph, thus they cannot check whether
//! the lanelets are actually connected to each other. In order not to break paths accidently, users are not allowed to manipulate path manually, but should use safe helper function instead.
class LaneletPath
{
  // for path surgery
  friend class LaneletRoute;
  friend class LaneletRouteBuilder;

public:
  using const_iterator = std::vector<LaneletSection>::const_iterator;
  using const_reverse_iterator = std::vector<LaneletSection>::const_reverse_iterator;

  //! @brief Uninitialized path (empty).
  LaneletPath() = default;
  //! @brief Initialize a path of length 0 (goal=start).
  explicit LaneletPath(const LaneletPoint & point);
  //! @brief Initialize a path from a single section
  explicit LaneletPath(const LaneletSection & section);

  // utils

  //! @brief Whether the path is a point (length = 0)
  [[nodiscard]] bool isPoint() const; 

  //! @brief Total length of path centerline from start to goal.
  [[nodiscard]] double length() const;

  //! @brief Whether given point is inside the path.
  [[nodiscard]] bool contains(const LaneletPoint & point) const;

  //! @brief Whether the path looks ok (sections are well connected, has no loop, etc.)
  [[nodiscard]] bool validate() const;

  // point query

  //! @brief Get start point on first lanelet.
  //! @return start point if path is valid, an invalid point otherwise.
  [[nodiscard]] LaneletPoint getStartPoint() const;

  //! @brief Get goal point on last lanelet
  //! @return goal point if path is valid, an invalid point otherwise.
  [[nodiscard]] LaneletPoint getGoalPoint() const;

  //! @brief Get path point at given arc length.
  //! If the arc length is outside range [0., length()], the first/last point of the path is returned instead.
  //! @note arc length is relative to the path, not the path lanelets (i.e. path start point has arc length of 0)
  //! @return the point at given arc length, an invalid point is path is not initialized.
  [[nodiscard]] LaneletPoint getPointAt(const double path_arc_length) const;

  //! @brief Get arc length of a point on the path
  //! @return the arc length if the point is on the path, nothing otherwise
  [[nodiscard]] std::optional<double> getPathArcLength(const LaneletPoint& point) const;

  // query within path

  //! @brief Get closest lanelet point to pose within path.
  //! @return closest lanelet point if found, an invalid point otherwise.
  [[nodiscard]] LaneletPoint getClosestLaneletPointWithinPath(
    const geometry_msgs::msg::Pose pose) const;

  // line query

  //! @brief Get center line of path
  [[nodiscard]] std::vector<lanelet::BasicPoint3d> getCenterline() const;
  //! @brief Get left bound of path
  [[nodiscard]] std::vector<lanelet::BasicPoint3d> getLeftBound() const;
  //! @brief Get right bound of path
  [[nodiscard]] std::vector<lanelet::BasicPoint3d> getRightBound() const;

  // editing functions

  //! @brief Split the path in two at given point.
  //! Splitting will fail if the point is not on the path.
  //! @note If the split point appears multiple times in the path, only its first occurence (from start point) is used.
  //! @param split_point the point used to split the path in two
  //! @param path_before output path from the initial path start point to the split point. If null, the data will be discarded
  //! @param path_after output path from the split point to the initial path goal point. If null, the data will be discarded
  //! @return whether split is successful
  [[nodiscard]] bool split(
    const LaneletPoint & split_point,
    LaneletPath * path_before,
    LaneletPath * path_after) const;

  //! @brief Concatenate two paths.
  //! The two paths must be connected directly, i.e. the goal point of the first path and the start point of
  //! the second path should be the same.
  //! If the two paths overlaps, the overlap will be remove using given strategy.
  //! @return the concatenated path if successful, an empty path otherwise.
  [[nodiscard]] static LaneletPath concatenate(
    const LaneletPath & first_path, 
    const LaneletPath & second_path,
    const OverlapRemovalStrategy overlap_removal_strategy = OverlapRemovalStrategy::SPLIT);

  //! @brief Truncate path between the two given points
  //! @note Has the same effect than splitting the path twice at start and goal point, and returning the middle path
  //! @return the truncated path is successful, an empty path otherwise
  [[nodiscard]] LaneletPath truncate(
    const LaneletPoint & start_point,
    const LaneletPoint & goal_point) const;

  //! @brief Shrink path on both sides, according to given margin
  //! @param front_margin removed forward length from the path start point
  //! @param back_margin removed backward length from the path goal point
  //! @note Similar to truncate, except we use distances instead of points
  //! @return the shrinked path is successful, an empty path otherwise
  [[nodiscard]] LaneletPath shrink(
    const double front_margin,
    const double back_margin) const;

  // View functions on sections container.

  [[nodiscard]] bool empty() const { return sections_.empty(); }
  [[nodiscard]] std::size_t size() const { return sections_.size(); }
  [[nodiscard]] const_iterator begin() const { return sections_.begin(); }
  [[nodiscard]] const_iterator end() const { return sections_.end(); }
  [[nodiscard]] const_reverse_iterator rbegin() const { return sections_.rbegin(); }
  [[nodiscard]] const_reverse_iterator rend() const { return sections_.rend(); }
  [[nodiscard]] const LaneletSection& front() const { return sections_.front(); }
  [[nodiscard]] const LaneletSection& back() const { return sections_.back(); }

  // Getters

  [[nodiscard]] const LaneletSections & sections() const { return sections_; }

private:

  // Path surgery functions
  // Enter the realm of breaking and fixing paths. Use with caution.

  //! @brief Initialize a path from given lanelets (unchecked).
  //! Assume input lanelets form a valid path. Otherwise you are doomed.
  //! @note Lanelets before start or after goal point will be removed from the path 
  explicit LaneletPath(
    const lanelet::ConstLanelets & lanelets, 
    const LaneletPoint & start_point, 
    const LaneletPoint & goal_point);
  //! @brief LaneletPath constructor from given sections (unchecked).
  //! Assume input sections form a valid path. Otherwise you are doomed.
  explicit LaneletPath(const LaneletSections sections)
    : sections_(sections) {}

  //! @brief Get all overlapped sections within the path
  //! @return all overlapped sections in order (from start to goal)
  [[nodiscard]] LaneletSections getOverlappedSections() const;

  //! @brief Whether path has overlap issues (need fix)
  [[nodiscard]] bool hasOverlap() const { return !getOverlappedSections().empty(); };

  //! @brief Fix overlapping path using given strategy
  //! @note This function can only fix "simple" cases, for example when path overlap at the beginning and the end.
  //! @return the path without overlap if successful, an empty path otherwise
  [[nodiscard]] LaneletPath fixOverlap(const OverlapRemovalStrategy overlap_removal_strategy) const;

  LaneletSections sections_;  //!< Sections composing the lanelet. Except first and last sections, all inner sections are whole lanelets
};

}  // namespace route_handler

#endif  // ROUTE_HANDLER__LANELET_PATH_HPP_
