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

#include <path/display.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace
{
bool validateFloats(const std::vector<geometry_msgs::msg::Point> & bound)
{
  for (auto && point : bound) {
    if (!rviz_common::validateFloats(point)) {
      return false;
    }
  }
  return true;
}
bool validateFloats(const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr & msg_ptr)
{
  return validateFloats(msg_ptr->left_bound) && validateFloats(msg_ptr->right_bound);
}
}  // namespace

namespace rviz_plugins
{
AutowarePathDisplay::AutowarePathDisplay()
{
  property_drivable_area_view_ = new rviz_common::properties::BoolProperty(
    "View Drivable Area", true, "", this, SLOT(updateVisualization()));
  property_drivable_area_alpha_ = new rviz_common::properties::FloatProperty(
    "Alpha", 0.999, "", property_drivable_area_view_, SLOT(updateVisualization()), this);
  property_drivable_area_alpha_->setMin(0.0);
  property_drivable_area_alpha_->setMax(1.0);
  property_drivable_area_color_ = new rviz_common::properties::ColorProperty(
    "Color", QColor(0, 148, 205), "", property_drivable_area_view_, SLOT(updateVisualization()),
    this);
  property_drivable_area_width_ = new rviz_common::properties::FloatProperty(
    "Width", 0.2f, "", property_drivable_area_view_, SLOT(updateVisualization()), this);
  property_drivable_area_width_->setMin(0.001);
}

AutowarePathDisplay::~AutowarePathDisplay()
{
  if (this->initialized()) {
    scene_manager_->destroyManualObject(left_bound_object_);
    scene_manager_->destroyManualObject(right_bound_object_);
  }
}

void AutowarePathDisplay::onInitialize()
{
  rviz_common::MessageFilterDisplay<
    autoware_auto_planning_msgs::msg::Path>::MFDClass::onInitialize();

  path_manual_object_ = this->scene_manager_->createManualObject();
  velocity_manual_object_ = this->scene_manager_->createManualObject();
  left_bound_object_ = scene_manager_->createManualObject();
  right_bound_object_ = scene_manager_->createManualObject();
  path_manual_object_->setDynamic(true);
  velocity_manual_object_->setDynamic(true);
  left_bound_object_->setDynamic(true);
  right_bound_object_->setDynamic(true);
  this->scene_node_->attachObject(path_manual_object_);
  this->scene_node_->attachObject(velocity_manual_object_);
  scene_node_->attachObject(left_bound_object_);
  scene_node_->attachObject(right_bound_object_);
}
void AutowarePathDisplay::reset()
{
  rviz_common::MessageFilterDisplay<autoware_auto_planning_msgs::msg::Path>::MFDClass::reset();
  path_manual_object_->clear();
  velocity_manual_object_->clear();
  left_bound_object_->clear();
  right_bound_object_->clear();
  for (size_t i = 0; i < velocity_texts_.size(); i++) {
    Ogre::SceneNode * node = velocity_text_nodes_.at(i);
    node->detachAllObjects();
    node->removeAndDestroyAllChildren();
    this->scene_manager_->destroySceneNode(node);
  }
  velocity_text_nodes_.clear();
  velocity_texts_.clear();
}

void AutowarePathDisplay::visualizeDrivableArea(
  autoware_auto_planning_msgs::msg::Path::ConstSharedPtr msg_ptr)
{
  if (!validateFloats(msg_ptr)) {
    this->setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message drivable area contained invalid floating point values (nans or infs)");
    return;
  }

  left_bound_object_->clear();
  right_bound_object_->clear();
  if (property_drivable_area_view_->getBool()) {
    visualizeBound(msg_ptr->left_bound, left_bound_object_);
    visualizeBound(msg_ptr->right_bound, right_bound_object_);
  }
}

void AutowarePathDisplay::visualizeBound(
  const std::vector<geometry_msgs::msg::Point> & bound, Ogre::ManualObject * bound_object)
{
  if (bound.size() < 2) {
    return;
  }

  bound_object->estimateVertexCount(bound.size() * 2);
  bound_object->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);

  Ogre::ColourValue color =
    rviz_common::properties::qtToOgre(property_drivable_area_color_->getColor());
  color.a = property_drivable_area_alpha_->getFloat();
  const auto line_width = property_drivable_area_width_->getFloat();

  for (size_t i = 0; i < bound.size(); ++i) {
    const auto & curr_p = i == bound.size() - 1 ? bound.at(i - 1) : bound.at(i);
    const auto & next_p = i == bound.size() - 1 ? bound.at(i) : bound.at(i + 1);
    const auto yaw = tier4_autoware_utils::calcAzimuthAngle(curr_p, next_p);
    const auto x_offset = static_cast<float>(line_width * 0.5 * std::sin(yaw));
    const auto y_offset = static_cast<float>(line_width * 0.5 * std::cos(yaw));
    auto target_lp = bound.at(i);
    target_lp.x = target_lp.x - x_offset;
    target_lp.y = target_lp.y + y_offset;
    target_lp.z = target_lp.z + 0.5;
    bound_object->position(target_lp.x, target_lp.y, target_lp.z);
    bound_object->colour(color);
    auto target_rp = bound.at(i);
    target_rp.x = target_rp.x + x_offset;
    target_rp.y = target_rp.y - y_offset;
    target_rp.z = target_rp.z + 0.5;
    bound_object->position(target_rp.x, target_rp.y, target_rp.z);
    bound_object->colour(color);
  }
  bound_object->end();
}
}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathWithLaneIdDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowarePathDisplay, rviz_common::Display)
PLUGINLIB_EXPORT_CLASS(rviz_plugins::AutowareTrajectoryDisplay, rviz_common::Display)
