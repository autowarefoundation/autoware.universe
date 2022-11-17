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

#include "route_handler/lanelet_section.hpp"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/primitives/LineString.h"
#include "lanelet2_core/primitives/Point.h"
#include "route_handler/lanelet_point.hpp"

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/Lanelet.h>

namespace route_handler
{

using lanelet::utils::to2D;

namespace {

lanelet::BasicPoint3d get3DPointFrom2DArcLength(const lanelet::ConstLineString3d & line, const double s)
{
  if (line.empty()) {
    return {};
  }
  double accumulated_distance2d = 0;
  lanelet::BasicPoint3d prev_pt = line.front();
  for (const lanelet::BasicPoint3d & pt : line) {
    const double distance2d = lanelet::geometry::distance2d(to2D(prev_pt), to2D(pt));
    if (accumulated_distance2d + distance2d > s) {
      const double ratio = (s - accumulated_distance2d) / distance2d;
      const auto interpolated_pt = prev_pt * (1 - ratio) + pt * ratio;
      return lanelet::BasicPoint3d{interpolated_pt.x(), interpolated_pt.y(), interpolated_pt.z()};
    }
    accumulated_distance2d += distance2d;
    prev_pt = pt;
  }
  return {};
}

std::vector<lanelet::BasicPoint3d> truncateLine(const lanelet::ConstLineString3d & line, const double s_start, const double s_end)  {
  std::vector<lanelet::BasicPoint3d> truncated_line;
  // interpolate 3d points from 2d 
  double s = 0;
  for (size_t i = 0; i < line.size(); i++) {
    const auto & pt = line[i];
    const lanelet::BasicPoint3d next_pt =
      (i + 1 < line.size()) ? line[i + 1] : line[i];
    const double distance = lanelet::geometry::distance2d(to2D(pt), to2D(next_pt));

    if (s < s_start && s + distance > s_start) {
      const auto p = get3DPointFrom2DArcLength(line, s_start);
      truncated_line.push_back(p);
    }
    if (s >= s_start && s <= s_end) {
      truncated_line.push_back(pt);
    }
    if (s < s_end && s + distance > s_end) {
      const auto p = get3DPointFrom2DArcLength(line, s_end);
      truncated_line.push_back(p);
    }
    s += distance;
  }
  return truncated_line;
}

} // namespace

LaneletSection::LaneletSection(const LaneletPoint &point)
{
  if (!point.isValid()) {
    return;
  }
  
  lanelet_ = point.lanelet();
  lanelet_length_ = point.lanelet_length();
  start_arc_length_ = point.arc_length();
  end_arc_length_ = point.arc_length();
}

LaneletSection::LaneletSection(
  const lanelet::ConstLanelet & lanelet,
  const std::optional<double> optional_start_arc_length,
  const std::optional<double> optional_end_arc_length)
{
  if (lanelet.id() == lanelet::InvalId) {
    return; // invalid lanelet
  }
  double lanelet_length = lanelet::geometry::length2d(lanelet);
  double clamped_start_arc_length = optional_start_arc_length ?
    std::clamp(*optional_start_arc_length, 0., lanelet_length) : 0.;
  double clamped_end_arc_length = optional_end_arc_length ?
    std::clamp(*optional_end_arc_length, 0., lanelet_length) : lanelet_length;

  if (clamped_start_arc_length > clamped_end_arc_length) {
    return; // invalid section
  }

  lanelet_ = lanelet;
  lanelet_length_ = lanelet_length;
  start_arc_length_ = clamped_start_arc_length;
  end_arc_length_ = clamped_end_arc_length;
}

bool LaneletSection::isValid() const
{
  return lanelet_.id() != lanelet::InvalId && start_arc_length_ <= end_arc_length_;
}

bool LaneletSection::isPoint() const {
  if (!isValid()) {
    return false;
  }
  return start_arc_length_ == end_arc_length_;
}

bool LaneletSection::coversWholeLanelet() const
{
  if (!isValid()) {
    return false;
  }
  return start_arc_length_ == 0. && end_arc_length_ == lanelet_length_;
}

double LaneletSection::length() const
{
  if (!isValid()) {
    return 0.;
  }
  return end_arc_length_ - start_arc_length_;
}

bool LaneletSection::contains(const LaneletPoint& point) const
{
  if (!isValid()) {
    return false;
  }
  if (point.lanelet() != lanelet_) {
    return false;
  }
  return point.arc_length() >= start_arc_length_
      && point.arc_length() <= end_arc_length_;
}

LaneletPoint LaneletSection::getStartPoint() const
{
  if (!isValid()) {
    return {};
  }
  return LaneletPoint{lanelet_, start_arc_length_};
}

LaneletPoint LaneletSection::getEndPoint() const
{
  if (!isValid()) {
    return {};
  }
  return LaneletPoint{lanelet_, end_arc_length_};
}

LaneletPoint LaneletSection::getPointAt(const double arc_length) const
{
  if (!isValid()) {
    return {};
  }
  if (arc_length < 0.) {
    return getStartPoint();
  }
  if (arc_length > length()) {
    return getEndPoint();
  }
  return LaneletPoint{lanelet_, start_arc_length_ + arc_length};
}

std::vector<lanelet::BasicPoint3d> LaneletSection::getCenterline() const
{
  if (!isValid()) {
    return {};
  }

  const auto & target_line = lanelet_.centerline();
  const double s_start = start_arc_length_;
  const double s_end = end_arc_length_;

  // interpolate 3d points from 2d 
  const auto centerline = truncateLine(target_line, s_start, s_end);
  return centerline;
}

std::vector<lanelet::BasicPoint3d> LaneletSection::getLeftBound() const
{
  if (!isValid()) {
    return {};
  }

  const auto & target_line = lanelet_.leftBound();
  const auto target_start = start_arc_length_ == 0 ?
    to2D(target_line.front()) : getStartPoint().toBasicPoint2d();
  const auto target_end = end_arc_length_ == lanelet_length_ ?
    to2D(target_line.back()) : getEndPoint().toBasicPoint2d();

  const double s_start = lanelet::geometry::toArcCoordinates(lanelet_.leftBound2d(), target_start).length;
  const double s_end = lanelet::geometry::toArcCoordinates(lanelet_.leftBound2d(), target_end).length;

  if (s_start > s_end) {
    return {};
  }

  // interpolate 3d points from 2d 
  const auto left_bound = truncateLine(target_line, s_start, s_end);
  return left_bound;
}

std::vector<lanelet::BasicPoint3d> LaneletSection::getRightBound() const
{
 if (!isValid()) {
    return {};
  }

  const auto & target_line = lanelet_.rightBound();
  const auto target_start = start_arc_length_ == 0 ?
    to2D(target_line.front()) : getStartPoint().toBasicPoint2d();
  const auto target_end = end_arc_length_ == lanelet_length_ ?
    to2D(target_line.back()) : getEndPoint().toBasicPoint2d();

  const double s_start = lanelet::geometry::toArcCoordinates(lanelet_.rightBound2d(), target_start).length;
  const double s_end = lanelet::geometry::toArcCoordinates(lanelet_.rightBound2d(), target_end).length;

  if (s_start > s_end) {
    return {};
  }

  // interpolate 3d points from 2d 
  const auto right_bound = truncateLine(target_line, s_start, s_end);
  return right_bound;
}


bool LaneletSection::split(
  const LaneletPoint & split_point,
  LaneletSection * section_before,
  LaneletSection * section_after) const
{
  if (!isValid()) {
    return false;
  }
  if (!contains(split_point)) {
    return false;
  }

  if (section_before) { 
    *section_before = LaneletSection{lanelet_, start_arc_length_, split_point.arc_length()};
  }
  if (section_after) {
    *section_after = LaneletSection{lanelet_, split_point.arc_length(), end_arc_length_};
  }

  return true;
}

LaneletSection LaneletSection::concatenate(
  const LaneletSection &first_section, 
  const LaneletSection &second_section)
{
  if (!first_section.isValid() || !second_section.isValid()) {
    return {};
  }
  if (first_section.lanelet_ != second_section.lanelet_) {
    return {}; // not same lanelet
  }
  if (first_section.end_arc_length_ != second_section.start_arc_length_) {
    return {}; // not connected
  }
  return LaneletSection{first_section.lanelet_, first_section.start_arc_length_, second_section.end_arc_length_};
}

LaneletSection LaneletSection::intersect(
  const LaneletSection & section_A,
  const LaneletSection & section_B)
{
  if (!section_A.isValid() || !section_B.isValid()) {
    return {};
  }
  if (section_A.lanelet_ != section_B.lanelet_) {
    return {}; // not same lanelet
  }
  double intersection_start = std::max(section_A.start_arc_length_, section_B.start_arc_length_);
  double intersection_end = std::min(section_A.end_arc_length_, section_B.end_arc_length_);
  return LaneletSection{section_A.lanelet_, intersection_start, intersection_end};
}

} // namespace route_handler