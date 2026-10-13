// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/avoidance_target_detector/boundary.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{
namespace
{

using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::TrajectoryPoint;

constexpr std::array<lanelet::Id, 3> k_main_ids{101, 102, 103};
constexpr std::array<lanelet::Id, 3> k_shoulder_ids{201, 202, 203};
constexpr lanelet::Id k_disconnected_id = 999;
constexpr lanelet::Id k_unknown_id = 123456;
constexpr double k_lanelet_length_m = 10.0;
constexpr double k_road_speed_mps = 13.8;
constexpr double k_low_speed_mps = 2.0;

// Test-only access to private characterization targets without changing the production header.
template <typename Tag, typename Tag::Type member>
struct PrivateMemberAccess
{
  friend typename Tag::Type get_private_member(Tag) { return member; }
};

struct FindSegmentForLaneletTag
{
  using Type = std::optional<std::size_t> (ExtendedRouteHandler::*)(lanelet::Id) const;
  friend Type get_private_member(FindSegmentForLaneletTag);
};

struct FindSegmentForPointTag
{
  using Type =
    std::optional<std::size_t> (ExtendedRouteHandler::*)(const geometry_msgs::msg::Point &) const;
  friend Type get_private_member(FindSegmentForPointTag);
};

struct GetNearSegmentsTag
{
  using Type = std::vector<ExtendedLaneletSegments::Segment> (ExtendedRouteHandler::*)(
    const geometry_msgs::msg::Point &, const geometry_msgs::msg::Point &) const;
  friend Type get_private_member(GetNearSegmentsTag);
};

template struct PrivateMemberAccess<
  FindSegmentForLaneletTag, &ExtendedRouteHandler::find_segment_index_for_lanelet>;
template struct PrivateMemberAccess<
  FindSegmentForPointTag, &ExtendedRouteHandler::find_segment_index_for_point>;
template struct PrivateMemberAccess<GetNearSegmentsTag, &ExtendedRouteHandler::get_near_segments>;

geometry_msgs::msg::Point make_point(const double x, const double y, const double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

void set_lanelet_attributes(
  lanelet::Lanelet & lanelet, const std::string & subtype,
  const std::optional<double> speed_limit = std::nullopt)
{
  lanelet.setAttribute(lanelet::AttributeName::Type, lanelet::AttributeValueString::Lanelet);
  lanelet.setAttribute(lanelet::AttributeName::Subtype, subtype);
  lanelet.setAttribute(lanelet::AttributeName::Location, lanelet::AttributeValueString::Urban);
  lanelet.setAttribute(lanelet::AttributeName::OneWay, "yes");
  if (speed_limit) {
    lanelet.setAttribute("speed_limit", std::to_string(*speed_limit));
  }
}

LaneletPrimitive make_primitive(const lanelet::Id id)
{
  LaneletPrimitive primitive;
  primitive.id = id;
  primitive.primitive_type = "lane";
  return primitive;
}

LaneletRoute make_route()
{
  LaneletRoute route;
  route.header.frame_id = "map";
  route.start_pose.position = make_point(0.5, 0.0);
  route.start_pose.orientation.w = 1.0;
  route.goal_pose.position = make_point(29.5, 0.0);
  route.goal_pose.orientation.w = 1.0;

  for (const auto id : k_main_ids) {
    LaneletSegment segment;
    segment.preferred_primitive = make_primitive(id);
    segment.primitives.push_back(segment.preferred_primitive);
    route.segments.push_back(segment);
  }
  return route;
}

lanelet::LaneletMapPtr make_map()
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  std::array<lanelet::Point3d, 4> main_left_points;
  std::array<lanelet::Point3d, 4> shared_points;
  std::array<lanelet::Point3d, 4> shoulder_right_points;

  for (std::size_t i = 0; i < main_left_points.size(); ++i) {
    const double x = static_cast<double>(i) * k_lanelet_length_m;
    main_left_points[i] = lanelet::Point3d(lanelet::utils::getId(), x, 2.0, 0.0);
    shared_points[i] = lanelet::Point3d(lanelet::utils::getId(), x, -2.0, 0.0);
    shoulder_right_points[i] = lanelet::Point3d(lanelet::utils::getId(), x, -4.0, 0.0);
  }

  for (std::size_t i = 0; i < k_main_ids.size(); ++i) {
    lanelet::LineString3d main_left{
      lanelet::utils::getId(), {main_left_points[i], main_left_points[i + 1]}};
    lanelet::LineString3d shared_bound{
      lanelet::utils::getId(), {shared_points[i], shared_points[i + 1]}};
    lanelet::LineString3d shoulder_right{
      lanelet::utils::getId(), {shoulder_right_points[i], shoulder_right_points[i + 1]}};

    lanelet::Lanelet main_lanelet{k_main_ids[i], main_left, shared_bound};
    set_lanelet_attributes(
      main_lanelet, lanelet::AttributeValueString::Road, k_road_speed_mps * 3.6);
    map->add(main_lanelet);

    lanelet::Lanelet shoulder_lanelet{k_shoulder_ids[i], shared_bound, shoulder_right};
    const auto shoulder_speed =
      i == 1 ? std::optional<double>{k_low_speed_mps * 3.6} : std::nullopt;
    set_lanelet_attributes(shoulder_lanelet, "road_shoulder", shoulder_speed);
    map->add(shoulder_lanelet);
  }

  lanelet::LineString3d disconnected_left{
    lanelet::utils::getId(),
    {lanelet::Point3d(lanelet::utils::getId(), 1000.0, 2.0, 0.0),
     lanelet::Point3d(lanelet::utils::getId(), 1010.0, 2.0, 0.0)}};
  lanelet::LineString3d disconnected_right{
    lanelet::utils::getId(),
    {lanelet::Point3d(lanelet::utils::getId(), 1000.0, -2.0, 0.0),
     lanelet::Point3d(lanelet::utils::getId(), 1010.0, -2.0, 0.0)}};
  lanelet::Lanelet disconnected{k_disconnected_id, disconnected_left, disconnected_right};
  set_lanelet_attributes(disconnected, lanelet::AttributeValueString::Road, k_road_speed_mps * 3.6);
  map->add(disconnected);

  return map;
}

class ExtendedRouteHandlerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    source_map_ = make_map();
    route_ = make_route();

    LaneletMapBin map_msg;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    lanelet::utils::conversion::toBinMsg(source_map_, &map_msg);
#pragma GCC diagnostic pop
    map_msg.header.frame_id = "map";

    handler_ = std::make_shared<ExtendedRouteHandler>(map_msg, route_);
    handler_->create_map();

    // create_map() intentionally selects only route primitives. Add the disconnected source
    // lanelet to the query layer so point lookup can characterize a nearest lanelet with no route
    // segment without including that lanelet in the route itself.
    handler_->getRouteMap()->add(source_map_->laneletLayer.get(k_disconnected_id));
  }

  std::optional<std::size_t> find_segment(const lanelet::Id id) const
  {
    return (handler_.get()->*get_private_member(FindSegmentForLaneletTag{}))(id);
  }

  std::optional<std::size_t> find_segment(const geometry_msgs::msg::Point & point) const
  {
    return (handler_.get()->*get_private_member(FindSegmentForPointTag{}))(point);
  }

  std::vector<ExtendedLaneletSegments::Segment> get_near_segments(
    const geometry_msgs::msg::Point & start, const geometry_msgs::msg::Point & end) const
  {
    return (handler_.get()->*get_private_member(GetNearSegmentsTag{}))(start, end);
  }

  lanelet::LaneletMapPtr source_map_;
  LaneletRoute route_;
  std::shared_ptr<ExtendedRouteHandler> handler_;
};

TEST_F(ExtendedRouteHandlerTest, FindSegmentIndexForLanelet_ValidPrimitives)
{
  EXPECT_EQ(find_segment(k_main_ids[0]), 0U);
  EXPECT_EQ(find_segment(k_main_ids[1]), 1U);
  EXPECT_EQ(find_segment(k_main_ids[2]), 2U);
  EXPECT_EQ(find_segment(k_shoulder_ids[1]), 1U);
}

TEST_F(ExtendedRouteHandlerTest, FindSegmentIndexForLanelet_UnknownIdReturnsNullopt)
{
  EXPECT_FALSE(find_segment(k_unknown_id));
  EXPECT_FALSE(find_segment(k_disconnected_id));
}

TEST_F(ExtendedRouteHandlerTest, FindSegmentIndexForPoint_ValidPointReturnsClosestSegment)
{
  EXPECT_EQ(find_segment(make_point(5.0, 0.0)), 0U);
  EXPECT_EQ(find_segment(make_point(25.0, 0.0, 1.0)), 2U);
}

TEST_F(ExtendedRouteHandlerTest, FindSegmentIndexForPoint_PointOutsideMapReturnsNullopt)
{
  EXPECT_FALSE(find_segment(make_point(1005.0, 0.0)));
}

TEST_F(ExtendedRouteHandlerTest, GetNearSegments_ForwardOrderReturnsSubrange)
{
  const auto segments = get_near_segments(make_point(5.0, 0.0), make_point(25.0, 0.0));
  ASSERT_EQ(segments.size(), 3U);
  EXPECT_EQ(segments.front().preferred_primitive, k_main_ids.front());
  EXPECT_EQ(segments.back().preferred_primitive, k_main_ids.back());
}

TEST_F(ExtendedRouteHandlerTest, GetNearSegments_ReversedOrderReturnsEmpty)
{
  EXPECT_TRUE(get_near_segments(make_point(25.0, 0.0), make_point(5.0, 0.0)).empty());
}

TEST_F(ExtendedRouteHandlerTest, GetNearSegmentPolygon_ValidRangeProducesClosedPolygon)
{
  const auto polygon =
    handler_->get_near_segment_polygon(make_point(5.0, 0.0), make_point(25.0, 0.0));

  ASSERT_GE(polygon.size(), 4U);
  EXPECT_DOUBLE_EQ(polygon.front().x(), polygon.back().x());
  EXPECT_DOUBLE_EQ(polygon.front().y(), polygon.back().y());
}

TEST_F(ExtendedRouteHandlerTest, GetNearSegmentPolygon_InvalidEndpointsReturnEmptyPolygon)
{
  EXPECT_TRUE(
    handler_->get_near_segment_polygon(make_point(1005.0, 0.0), make_point(25.0, 0.0)).empty());
  EXPECT_TRUE(
    handler_->get_near_segment_polygon(make_point(5.0, 0.0), make_point(1005.0, 0.0)).empty());
}

TEST_F(ExtendedRouteHandlerTest, GetPrimitiveSetBounds_ValidChainReturnsLeftAndRightLinestrings)
{
  const auto bounds = handler_->get_primitive_set_bounds({k_main_ids[1], k_shoulder_ids[1]});

  ASSERT_FALSE(bounds.first.empty());
  ASSERT_FALSE(bounds.second.empty());
  EXPECT_DOUBLE_EQ(bounds.first.front().y(), 2.0);
  EXPECT_DOUBLE_EQ(bounds.second.front().y(), -4.0);
}

TEST_F(ExtendedRouteHandlerTest, GetPrimitiveSetBounds_UnknownIdReturnsEmptyPair)
{
  const auto bounds = handler_->get_primitive_set_bounds({k_unknown_id});
  EXPECT_TRUE(bounds.first.empty());
  EXPECT_TRUE(bounds.second.empty());
}

TEST_F(ExtendedRouteHandlerTest, BuildRouteBounds_CombinesSegmentsIntoCompoundBounds)
{
  const auto & original_bounds = handler_->get_original_route_bounds();
  const auto & extended_bounds = handler_->get_extended_route_bounds();

  ASSERT_FALSE(original_bounds.first.empty());
  ASSERT_FALSE(original_bounds.second.empty());
  ASSERT_FALSE(extended_bounds.first.empty());
  ASSERT_FALSE(extended_bounds.second.empty());
  EXPECT_DOUBLE_EQ(original_bounds.first.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(original_bounds.first.back().x(), 30.0);
  EXPECT_DOUBLE_EQ(extended_bounds.second.front().x(), 0.0);
  EXPECT_DOUBLE_EQ(extended_bounds.second.back().x(), 30.0);
}

TEST_F(ExtendedRouteHandlerTest, GetVelocityLimit_VehicularLaneReturnsAttributeLimit)
{
  const auto velocity_limit = handler_->get_velocity_limit(lanelet::BasicPoint2d{5.0, 0.0});
  ASSERT_TRUE(velocity_limit);
  EXPECT_DOUBLE_EQ(*velocity_limit, k_road_speed_mps);
}

TEST_F(ExtendedRouteHandlerTest, GetVelocityLimit_ShoulderLaneFallsBackToAdjacentRoadSpeed)
{
  const auto velocity_limit = handler_->get_velocity_limit(make_point(5.0, -3.0));
  ASSERT_TRUE(velocity_limit);
  EXPECT_DOUBLE_EQ(*velocity_limit, k_road_speed_mps);
}

TEST_F(ExtendedRouteHandlerTest, GetVelocityLimit_MultipleNearbyLaneletsReturnMinimumLimit)
{
  const lanelet::Point2d point{lanelet::utils::getId(), 15.0, -2.0};
  const auto velocity_limit = handler_->get_velocity_limit(point);
  ASSERT_TRUE(velocity_limit);
  EXPECT_DOUBLE_EQ(*velocity_limit, k_low_speed_mps);
}

TEST_F(ExtendedRouteHandlerTest, GetVelocityLimit_DebugOverrideReplacesLaneletAttribute)
{
  const ExtendedRouteHandler::VelocityLimitOverrides overrides{{k_main_ids[0], 4.0}};
  const auto velocity_limit = handler_->get_velocity_limit(make_point(5.0, 0.0), overrides);

  ASSERT_TRUE(velocity_limit);
  EXPECT_DOUBLE_EQ(*velocity_limit, 4.0);
}

TEST_F(ExtendedRouteHandlerTest, GetVelocityLimit_DebugOverridesStillUseNearbyMinimum)
{
  const ExtendedRouteHandler::VelocityLimitOverrides overrides{
    {k_main_ids[1], 10.0}, {k_shoulder_ids[1], 4.0}};
  const auto velocity_limit = handler_->get_velocity_limit(make_point(15.0, -2.0), overrides);

  ASSERT_TRUE(velocity_limit);
  EXPECT_DOUBLE_EQ(*velocity_limit, 4.0);
}

TEST_F(ExtendedRouteHandlerTest, ToPathMsg_PopulatesBoundsAndKinematicsCorrectly)
{
  lanelet::LineString2d left_bound{lanelet::utils::getId()};
  left_bound.push_back(lanelet::Point2d(lanelet::utils::getId(), 0.0, 2.0));
  left_bound.push_back(lanelet::Point2d(lanelet::utils::getId(), 10.0, 2.0));
  lanelet::LineString2d right_bound{lanelet::utils::getId()};
  right_bound.push_back(lanelet::Point2d(lanelet::utils::getId(), 0.0, -2.0));
  right_bound.push_back(lanelet::Point2d(lanelet::utils::getId(), 10.0, -2.0));

  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp.sec = 42;
  for (std::size_t i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position = make_point(static_cast<double>(i), 0.1 * static_cast<double>(i));
    point.pose.orientation.z = 0.01 * static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 1.0F + static_cast<float>(i);
    point.lateral_velocity_mps = 0.1F * static_cast<float>(i);
    point.heading_rate_rps = 0.01F * static_cast<float>(i);
    trajectory.points.push_back(point);
  }

  const auto path = to_path_msg({left_bound, right_bound}, trajectory);

  EXPECT_EQ(path.header, trajectory.header);
  EXPECT_EQ(path.left_bound.size(), left_bound.size());
  EXPECT_EQ(path.right_bound.size(), right_bound.size());
  ASSERT_EQ(path.points.size(), trajectory.points.size());
  for (std::size_t i = 0; i < path.points.size(); ++i) {
    EXPECT_EQ(path.points[i].pose, trajectory.points[i].pose);
    EXPECT_FLOAT_EQ(
      path.points[i].longitudinal_velocity_mps, trajectory.points[i].longitudinal_velocity_mps);
    EXPECT_FLOAT_EQ(path.points[i].lateral_velocity_mps, trajectory.points[i].lateral_velocity_mps);
    EXPECT_FLOAT_EQ(path.points[i].heading_rate_rps, trajectory.points[i].heading_rate_rps);
  }
}

TEST_F(ExtendedRouteHandlerTest, GetPrimitiveSetBounds_EmptyInputReturnsEmptyPair)
{
  // Guards against Undefined Behavior when accessing .front() / .back() on empty vectors
  const auto bounds = handler_->get_primitive_set_bounds({});
  EXPECT_TRUE(bounds.first.empty());
  EXPECT_TRUE(bounds.second.empty());
}

}  // namespace
}  // namespace autoware::avoidance_target_detector
