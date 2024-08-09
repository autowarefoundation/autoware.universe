// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_velocity_planner_common/planner_data.hpp"

#include <boost/geometry/geometry.hpp>

#include <gtest/gtest.h>

using autoware::motion_velocity_planner::Collision;
using autoware::motion_velocity_planner::CollisionChecker;
using autoware::universe_utils::Line2d;
using autoware::universe_utils::MultiLineString2d;
using autoware::universe_utils::MultiPoint2d;
using autoware::universe_utils::MultiPolygon2d;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

TEST(TestPlannerData, CollisionChecker)
{
}
