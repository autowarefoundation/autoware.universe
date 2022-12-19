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

#include "route_handler/lanelet_point.hpp"

#include "lanelet2_core/primitives/Point.h"
#include "route_handler/lanelet_section.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

namespace route_handler
{

LaneletPoint::LaneletPoint(const lanelet::ConstLanelet & lanelet, double arc_length)
{
  if (lanelet.id() != lanelet::InvalId) {
    lanelet_ = lanelet;
    lanelet_length_ = lanelet::geometry::length2d(lanelet);
    arc_length_ = std::clamp(arc_length, 0.0, lanelet_length_);
  }
}

LaneletPoint LaneletPoint::startOf(const lanelet::ConstLanelet & lanelet)
{
  if (lanelet.id() == lanelet::InvalId) {
    return LaneletPoint{};
  }
  return LaneletPoint{lanelet, 0.};
}

LaneletPoint LaneletPoint::endOf(const lanelet::ConstLanelet & lanelet)
{
  if (lanelet.id() == lanelet::InvalId) {
    return LaneletPoint{};
  }
  return LaneletPoint{lanelet, lanelet::geometry::length2d(lanelet)};
}

LaneletPoint LaneletPoint::fromProjection(
  const lanelet::ConstLanelet & lanelet, const lanelet::BasicPoint2d & point)
{
  if (lanelet.id() == lanelet::InvalId) {
    return LaneletPoint{};
  }
  const double arc_length =
    lanelet::geometry::toArcCoordinates(lanelet.centerline2d(), point).length;
  return LaneletPoint(lanelet, arc_length);
}

LaneletPoint LaneletPoint::fromProjection(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Point & point)
{
  const lanelet::BasicPoint2d basic_point(point.x, point.y);
  return fromProjection(lanelet, basic_point);
}

LaneletPoint LaneletPoint::fromProjection(
  const lanelet::ConstLanelet & lanelet, const geometry_msgs::msg::Pose & pose)
{
  return fromProjection(lanelet, pose.position);
}

lanelet::BasicPoint2d LaneletPoint::toBasicPoint2d() const
{
  lanelet::BasicPoint2d basic_point;
  if (!isValid()) {
    return basic_point;
  }
  basic_point =
    lanelet::geometry::interpolatedPointAtDistance(lanelet_.centerline2d(), arc_length_);
  return basic_point;
}

geometry_msgs::msg::Point LaneletPoint::toPointMsg() const
{
  geometry_msgs::msg::Point point_msg;
  if (!isValid()) {
    return point_msg;
  }
  const lanelet::BasicPoint2d basic_point = toBasicPoint2d();
  point_msg.x = basic_point.x();
  point_msg.y = basic_point.y();
  point_msg.z = 0;
  return point_msg;
}

bool LaneletPoint::isValid() const { return lanelet_.id() != lanelet::InvalId; }

LaneletSection LaneletPoint::forwardSection() const
{
  if (!isValid()) {
    return {};
  }

  return LaneletSection{lanelet_, arc_length_, {}};
}

LaneletSection LaneletPoint::backwardSection() const
{
  if (!isValid()) {
    return {};
  }

  return LaneletSection{lanelet_, {}, arc_length_};
}

}  // namespace route_handler
