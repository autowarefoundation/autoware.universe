// Copyright 2020 Tier IV, Inc.
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

#ifndef POSE_HISTORY_HPP
#define POSE_HISTORY_HPP

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "rviz/display.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/billboard_line.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/color_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/ros_topic_property.h"
//#include "rviz/ogre_helpers/arrow.h"
#include <deque>
#include <memory>

namespace rviz_plugins
{
class PoseHistory : public rviz::Display
{
  Q_OBJECT

public:
  PoseHistory();
  virtual ~PoseHistory();

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:

  void updateTopic();
  void subscribe();
  void unsubscribe();
  void onMessage(const geometry_msgs::PoseStamped & message);

private:
  void updateHistory();
  void updateLines();
  // void updateArrows();

private:
  std::string target_frame_;
  std::deque<geometry_msgs::PoseStamped> history_;
  std::unique_ptr<rviz::BillboardLine> lines_;
  // std::deque<std::unique_ptr<rviz::Arrow>> arrows_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  rviz::RosTopicProperty * property_topic_;
  rviz::IntProperty * property_buffer_size_;
  rviz::BoolProperty * property_line_view_;
  rviz::FloatProperty * property_line_width_;
  rviz::ColorProperty * property_line_color_;
};

}  // namespace rviz_plugins

#endif
