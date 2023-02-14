// Copyright 2023 TIER IV, Inc.
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

#ifndef PATH__DISPLAY_HPP_
#define PATH__DISPLAY_HPP_

#include <path/display_base.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <vector>

namespace rviz_plugins
{
class AutowarePathWithLaneIdDisplay
: public AutowarePathBaseDisplay<autoware_auto_planning_msgs::msg::PathWithLaneId>
{
  Q_OBJECT
};

class AutowarePathDisplay : public AutowarePathBaseDisplay<autoware_auto_planning_msgs::msg::Path>
{
  Q_OBJECT
public:
  AutowarePathDisplay();
  ~AutowarePathDisplay();
  void onInitialize() override;
  void reset() override;

protected:
  void visualizeDrivableArea(
    autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg_ptr) override;
  void visualizeBound(
    const std::vector<geometry_msgs::msg::Point> & bound, Ogre::ManualObject * bound_object);

private:
  Ogre::ManualObject * left_bound_object_{nullptr};
  Ogre::ManualObject * right_bound_object_{nullptr};

  rviz_common::properties::BoolProperty * property_drivable_area_view_;
  rviz_common::properties::ColorProperty * property_drivable_area_color_;
  rviz_common::properties::FloatProperty * property_drivable_area_alpha_;
  rviz_common::properties::FloatProperty * property_drivable_area_width_;
};

class AutowareTrajectoryDisplay
: public AutowarePathBaseDisplay<autoware_auto_planning_msgs::msg::Trajectory>
{
  Q_OBJECT
};
}  // namespace rviz_plugins

#endif  // PATH__DISPLAY_HPP_
