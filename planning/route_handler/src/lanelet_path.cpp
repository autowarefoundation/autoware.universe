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

#include "route_handler/lanelet_path.hpp"
#include "lanelet2_core/geometry/Point.h"
#include "route_handler/lanelet_point.hpp"
#include "route_handler/lanelet_section.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>

namespace internal {

using route_handler::LaneletPoint;
using route_handler::LaneletSection;
using route_handler::LaneletSections;

// helper for building path section
LaneletSections buildSections(
  const lanelet::ConstLanelets & lanelets,
  const LaneletPoint & start_point,
  const LaneletPoint & goal_point)
{
  if (lanelets.empty()) {
    return {};
  }

  // find start and goal lanelets
  auto start_it = std::find(lanelets.begin(), lanelets.end(), start_point.lanelet());
  auto goal_it = std::find(lanelets.rbegin(), lanelets.rend(), goal_point.lanelet());

  if (start_it == lanelets.end() || goal_it == lanelets.rend()) {
    return {}; // points are not in the list
  }

  if (start_it > goal_it.base()) {
    return {}; // goal lanelet is before start
  }

  LaneletSections sections;
  const auto nb_sections = goal_it.base() - start_it;
  sections.reserve(nb_sections);

  if (nb_sections == 1) {
    // special case: only 1 lanelet -> start and goal are part of the same section
    LaneletSection section {*start_it, {start_point.arc_length()}, {goal_point.arc_length()}};
    if (section.isValid()) {
      sections.push_back(section);
    }
    return sections;
  }

  // nb_sections >= 2: start and goal are on different sections

  LaneletSection section;

  // start sections
  section = LaneletSection{*start_it, {start_point.arc_length()}, {}};
  if (!section.isValid()) {
    return {};
  }
  sections.push_back(section);

  // middle sections
  for (auto llt_it = start_it+1; llt_it != goal_it.base()-1; ++llt_it) {
    section = LaneletSection{*llt_it};
    if (!section.isValid()) {
      return {};
    }
    sections.push_back(section); // whole lanelet
  }

  // goal section
  section = LaneletSection{*goal_it, {}, {goal_point.arc_length()}};
  if (!section.isValid()) {
    return {};
  }
  sections.push_back(section);

  return sections;
}

} // namespace internal

namespace route_handler
{

LaneletPath::LaneletPath(const LaneletPoint & point)
{
  if (point.isValid()) {
    sections_.clear();
    sections_.push_back(LaneletSection{
      point.lanelet(),
      point.arc_length(),
      point.arc_length()});
  }
}

LaneletPath::LaneletPath(const LaneletSection & section)
{
  if (section.isValid()) {
    sections_.clear();
    sections_.push_back(section);
  }
}

LaneletPath::LaneletPath(
  const lanelet::ConstLanelets & lanelets, 
  const LaneletPoint & start_point, 
  const LaneletPoint & goal_point)
{
  sections_ = internal::buildSections(lanelets, start_point, goal_point);
}

bool LaneletPath::isPoint() const
{
  return sections_.size() == 1 && sections_.front().isPoint();
}

double LaneletPath::length() const
{
  double l = 0.;
  for (const auto & section : sections_) {
    l += section.length();
  }

  return l;
}

bool LaneletPath::contains(const LaneletPoint & point) const
{
  return std::any_of(sections_.begin(), sections_.end(),
    [&](const LaneletSection& section) { return section.contains(point); }
  );
}

bool LaneletPath::validate() const
{
  if (sections_.empty()) {
    return false;
  }

  if (sections_.size() == 1) {
    // single section path
    return sections_.front().isValid();
  }

  // checking sections
  if (sections_.front().end_arc_length() != sections_.front().lanelet_length()) {
    return false; // first section is not connected with others
  }
  for (auto it = sections_.begin()+1; it != sections_.end()-1; ++it) {
    if (!it->coversWholeLanelet()) {
      return false; // middle section not connected with previous or next
    }
  }
  if (sections_.back().start_arc_length() != 0.) {
    return false; // last section is not connected with others
  }

  // checking loops
  for (auto it = sections_.begin(); it != sections_.end(); ++it) {
    // all section pairs within the path should be disjoint (not even 1 point in common)
    if (std::any_of(sections_.begin(), it,
      [&](const auto & section) {
        return LaneletSection::intersect(section, *it).isValid();
      })) {
      return false; // found loop
    }
  }

  // everything looks ok
  return true;
}

LaneletPoint LaneletPath::getStartPoint() const
{
  if (sections_.empty()) {
    return {};
  }
  return sections_.front().getStartPoint();
}

LaneletPoint LaneletPath::getGoalPoint() const
{
  if (sections_.empty()) {
    return {};
  }
  return sections_.front().getEndPoint();
}

LaneletPoint LaneletPath::getPointAt(const double path_arc_length) const
{
  if (sections_.empty()) {
    return {};
  }
  
  double remaining_distance = path_arc_length;
  for (auto & section: sections_) {
    if (remaining_distance <= section.length()) {
      return section.getPointAt(remaining_distance); 
    }
    remaining_distance -= section.length();
  }

  return getGoalPoint();
}

std::optional<double> LaneletPath::getPathArcLength(const LaneletPoint & point) const
{
  if (sections_.empty()) {
    return {};
  }

  // find section containing the point
  const auto section_it = std::find_if(sections_.begin(), sections_.end(),
    [&](const LaneletSection& section) { 
      return section.contains(point); 
    });
  if (section_it == sections_.end()) {
    return {}; // not on the path
  }

  double path_arc_length = 0.;
  for (auto it = sections_.begin(); it != section_it; ++it) {
    path_arc_length += it->length();
  }
  path_arc_length += point.arc_length();

  return {path_arc_length};
}

LaneletPoint LaneletPath::getClosestLaneletPointWithinPath(
  const geometry_msgs::msg::Pose pose) const
{
  lanelet::ConstLanelets path_lanelets;
  path_lanelets.reserve(sections_.size());
  for (const auto& section: sections_) {
    path_lanelets.push_back(section.lanelet());
  }

  lanelet::ConstLanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(path_lanelets, pose, &closest_lanelet)) {
    return {};
  }
  auto closest_lanelet_point = LaneletPoint::fromProjection(closest_lanelet, pose);
  return closest_lanelet_point;
}

std::vector<lanelet::BasicPoint3d> LaneletPath::getCenterline() const
{
  if (sections_.empty()) {
    return {};
  }

  constexpr double eps = 1.0e-3;
  std::vector<lanelet::BasicPoint3d> centerline;
  for (const auto& section: sections_) {
    const auto section_centerline = section.getCenterline();
    if (!centerline.empty() && !section_centerline.empty() &&
      lanelet::geometry::distance3d(centerline.back(), section_centerline.front()) < eps) {
      // skip first point
      centerline.insert(centerline.end(), section_centerline.begin()+1, section_centerline.end());
    } else {
      centerline.insert(centerline.end(), section_centerline.begin(), section_centerline.end());
    }
  }
  return centerline;
}

std::vector<lanelet::BasicPoint3d> LaneletPath::getLeftBound() const
{
  if (sections_.empty()) {
    return {};
  }

  constexpr double eps = 1.0e-3;
  std::vector<lanelet::BasicPoint3d> left_bound;
  for (const auto& section: sections_) {
    const auto section_left_bound = section.getLeftBound();
    if (!left_bound.empty() && !section_left_bound.empty() &&
      lanelet::geometry::distance3d(left_bound.back(), section_left_bound.front()) < eps) {
      // skip first point
      left_bound.insert(left_bound.end(), section_left_bound.begin()+1, section_left_bound.end());
    } else {
      left_bound.insert(left_bound.end(), section_left_bound.begin(), section_left_bound.end());
    }
  }
  return left_bound;
}

std::vector<lanelet::BasicPoint3d> LaneletPath::getRightBound() const
{
  if (sections_.empty()) {
    return {};
  }

  constexpr double eps = 1.0e-3;
  std::vector<lanelet::BasicPoint3d> right_bound;
  for (const auto& section: sections_) {
    const auto section_right_bound = section.getRightBound();
    if (!right_bound.empty() && !section_right_bound.empty() &&
      lanelet::geometry::distance3d(right_bound.back(), section_right_bound.front()) < eps) {
      // skip first point
      right_bound.insert(right_bound.end(), section_right_bound.begin()+1, section_right_bound.end());
    } else {
      right_bound.insert(right_bound.end(), section_right_bound.begin(), section_right_bound.end());
    }
  }
  return right_bound;
}

bool LaneletPath::split(
    const LaneletPoint & split_point,
    LaneletPath * path_before,
    LaneletPath * path_after) const
{
  if (sections_.empty()) {
    return false;
  }

  // find split point location in the path
  auto split_section_it = std::find_if(sections_.begin(), sections_.end(),
    [&](const LaneletSection& section) { 
      return section.contains(split_point); 
    });
  if (split_section_it == sections_.end()) {
    return false; // not on the path
  }

  // split the section
  LaneletSection section_before;
  LaneletSection section_after;
  if (!split_section_it->split(split_point, &section_before, &section_after)) {
    return false; // should not happen
  }

  // extract path before split point
  if (path_before) {
    path_before->sections_.clear();
    path_before->sections_.insert(path_before->sections_.end(), sections_.begin(), split_section_it);
    path_before->sections_.push_back(section_before);
  }

  // extract path after split point
  if (path_after) {
    path_after->sections_.clear();
    path_after->sections_.push_back(section_after);
    path_after->sections_.insert(path_after->sections_.end(), split_section_it+1, sections_.end());
  }

  return true;
}

LaneletPath LaneletPath::concatenate(
  const LaneletPath & first_path, 
  const LaneletPath & second_path,
  const OverlapRemovalStrategy overlap_removal_strategy)
{
  if (first_path.sections_.empty() || second_path.sections_.empty()) {
    return {}; // invalid paths
  }

  // try to connect both ends
  LaneletSection joint_section = LaneletSection::concatenate(first_path.sections_.back(), second_path.sections_.front());
  if (!joint_section.isValid()) {
    return {}; // paths are not connected
  }

  LaneletSections concatenated_sections;
  concatenated_sections.reserve(first_path.size() + second_path.size());

  // add first path sections except last
  if (!first_path.sections_.empty()) {
    concatenated_sections.insert(
      concatenated_sections.end(),
      first_path.sections_.begin(),
      first_path.sections_.end()-1);
  }

  // add joint section
  concatenated_sections.push_back(joint_section);

  // add second path sections except first
  if (!second_path.sections_.empty()) {
    concatenated_sections.insert(
      concatenated_sections.end(),
      second_path.sections_.begin()+1,
      second_path.sections_.end());
  }

  LaneletPath concatenated_path{concatenated_sections};

  // fix eventual overlap
  LaneletPath fixed_path = concatenated_path.fixOverlap(overlap_removal_strategy);

  return fixed_path;
}

LaneletPath LaneletPath::truncate(
  const LaneletPoint & start_point,
  const LaneletPoint & goal_point) const
{
  if (sections_.empty()) {
    return {};
  }

  if (!start_point.isValid() || !goal_point.isValid()) {
    return {};
  }

  LaneletPath path_from_start;
  if (!split(start_point, nullptr, &path_from_start)) {
    return {};
  }

  LaneletPath truncated_path;
  if (!path_from_start.split(goal_point, &truncated_path, nullptr)) {
    return {};
  }

  return truncated_path;
}

LaneletPath LaneletPath::shrink(
  const double front_margin,
  const double back_margin) const
{
  const auto start_point = getPointAt(front_margin);
  const auto goal_point = getPointAt(length() - back_margin);
  return truncate(start_point, goal_point);
}

LaneletSections LaneletPath::getOverlappedSections() const
{
  LaneletSections overlapping_sections;
  for (auto curr_it = sections_.begin(); curr_it != sections_.end(); ++curr_it) {
    for (auto prev_it = sections_.begin(); prev_it != curr_it; ++prev_it) {
      auto intersection = LaneletSection::intersect(*curr_it, *prev_it);
      if (intersection.length() > 0) {
        overlapping_sections.push_back(intersection);
      }
    }
  }
  return overlapping_sections;
}

LaneletPath LaneletPath::fixOverlap(
  const OverlapRemovalStrategy overlap_removal_strategy) const
{
  if (sections_.empty()) {
    return {};
  }

  if (overlap_removal_strategy == OverlapRemovalStrategy::DO_NOTHING) {
    return *this;
  }

  // find overlapped sections within path
  LaneletSections overlapped_sections = getOverlappedSections();

  if (overlapped_sections.empty()) {
    return *this; // nothing to fix
  }

  // Unless something is really wrong with the path, overlapped sections should form a path
  LaneletPath overlapped_path = LaneletPath{overlapped_sections};
  
  // But just to make sure... 
  if (!overlapped_path.validate()) {
    std::cerr << "Found overlapped sections do not form a valid path" << std::endl;
    return {};
  }

  // extract path with only the non-overlapped section
  LaneletPath path_without_overlap = truncate(overlapped_path.getGoalPoint(), overlapped_path.getStartPoint());
  if (path_without_overlap.empty()) {
    std::cerr << "Failed to extract non-overlapped path" << std::endl;
  }

  // overlapped path must be trimmed slightly, otherwise the fixed path will be a perfect loop (start=goal) 
  constexpr double eps = 1.e-3;
  LaneletPath fixed_path;
  switch (overlap_removal_strategy) {
    case OverlapRemovalStrategy::DO_NOTHING:
      return *this; // unreachable
    case OverlapRemovalStrategy::DISCARD:
      fixed_path = path_without_overlap;
      break;
    case OverlapRemovalStrategy::KEEP_START:
    {
      // shrink to prevent loops
      overlapped_path = overlapped_path.shrink(eps, 0.);
      // add the overlapped section at the beginning
      LaneletPath extended_path = LaneletPath::concatenate(overlapped_path, path_without_overlap, OverlapRemovalStrategy::DO_NOTHING);
      fixed_path = extended_path;
      break;
    }
    case OverlapRemovalStrategy::KEEP_END:
    {
      // shrink to prevent loops
      overlapped_path = overlapped_path.shrink(0., eps);
      // add the overlapped section at the end
      LaneletPath extended_path = LaneletPath::concatenate(path_without_overlap, overlapped_path, OverlapRemovalStrategy::DO_NOTHING);
      fixed_path = extended_path;
      break;
    }
    case OverlapRemovalStrategy::SPLIT:
    {
      // split overlapped path in the middle
      LaneletPoint split_point = overlapped_path.getPointAt(overlapped_path.length() / 2.);
      LaneletPath overlapped_first_half;
      LaneletPath overlapped_second_half;
      if (!overlapped_path.split(split_point, &overlapped_first_half, &overlapped_second_half)) {
        std::cerr << "Failed to split overlapped path evenly" << std::endl;
        return {};
      }
      // shrink to prevent loops
      overlapped_first_half = overlapped_first_half.shrink(eps, 0.);
      overlapped_second_half = overlapped_second_half.shrink(0., eps);
      // extend both sides of the path
      LaneletPath extended_path = LaneletPath::concatenate(overlapped_first_half, path_without_overlap, OverlapRemovalStrategy::DO_NOTHING);
      extended_path = LaneletPath::concatenate(extended_path, overlapped_second_half, OverlapRemovalStrategy::DO_NOTHING);
      fixed_path = extended_path;
      break;
    }
  }

  if (!fixed_path.validate()) {
    std::cerr << "Failed to fix overlap in the path" << std::endl;
    return {};
  }

  return fixed_path;
}

}  // namespace route_handler
