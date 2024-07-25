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

#include "pose_with_covariance_history_display.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/billboard_line.hpp>
#include <rviz_rendering/objects/shape.hpp>
#include <rviz_rendering/objects/arrow.hpp>

#include <tf2/LinearMath/Quaternion.h>

namespace rviz_plugins
{
PoseWithCovarianceHistory::PoseWithCovarianceHistory() : last_stamp_(0, 0, RCL_ROS_TIME)
{
  property_shape_view_ = new rviz_common::properties::BoolProperty("Covariance", true, "", this);
  property_shape_scale_ =
    new rviz_common::properties::FloatProperty("Scale", 1.0, "", property_shape_view_);
  property_shape_alpha_ =
    new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_shape_view_);
  property_shape_alpha_->setMin(0.0);
  property_shape_alpha_->setMax(1.0);
  property_shape_color_ =
    new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_shape_view_);

  property_shape_type_ = new rviz_common::properties::EnumProperty("Shape Type", "Line", "", this);
  property_shape_type_->addOption("Line", 0);
  property_shape_type_->addOption("Arrow", 1);

  property_buffer_size_ = new rviz_common::properties::IntProperty("Buffer Size", 100, "", this);
  
  //property_line_view_ = new rviz_common::properties::BoolProperty("Line", true, "", this);
  property_line_width_ =
    new rviz_common::properties::FloatProperty("Width", 0.1, "", property_shape_type_);
  property_line_alpha_ =
    new rviz_common::properties::FloatProperty("Alpha", 0.5, "", property_shape_type_);
  property_line_alpha_->setMin(0.0);
  property_line_alpha_->setMax(1.0);
  property_line_color_ =
    new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_shape_type_);
  
  //property_arrow_view_ = new rviz_common::properties::BoolProperty("Arrow", true, "", this);
  property_arrow_shaft_length_ =
    new rviz_common::properties::FloatProperty("Shaft Length", 0.6, "", property_shape_type_);
  property_arrow_shaft_diameter_ =
    new rviz_common::properties::FloatProperty("Shaft Radius", 0.3, "", property_shape_type_);
  property_arrow_head_length_ =
    new rviz_common::properties::FloatProperty("Head Length", 0.4, "", property_shape_type_);
  property_arrow_head_diameter_ =
    new rviz_common::properties::FloatProperty("Head Radius", 0.6, "", property_shape_type_);
  property_arrow_alpha_ =
    new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_shape_type_);
  property_arrow_alpha_->setMin(0.0);
  property_arrow_alpha_->setMax(1.0);
  property_arrow_color_ =
    new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_shape_type_);
  
  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(10000);
  property_line_width_->setMin(0.0);
  property_shape_scale_->setMin(0.0);
  property_shape_scale_->setMax(1000);
  property_arrow_shaft_length_->setMin(0.0);
  property_arrow_shaft_diameter_->setMin(0.0);
  property_arrow_head_length_->setMin(0.0);
  property_arrow_head_diameter_->setMin(0.0);
}

PoseWithCovarianceHistory::~PoseWithCovarianceHistory() = default;  // Properties are deleted by Qt

void PoseWithCovarianceHistory::onInitialize()
{
  MFDClass::onInitialize();
  lines_ = std::make_unique<rviz_rendering::BillboardLine>(scene_manager_, scene_node_);
}

void PoseWithCovarianceHistory::onEnable()
{
  subscribe();
}

void PoseWithCovarianceHistory::onDisable()
{
  unsubscribe();
}

void PoseWithCovarianceHistory::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  if (!history_.empty()) {
    lines_->clear();
    spheres_.clear();
    if (property_shape_view_->getBool()) {
      updateShapes();
    }
  }
}

void PoseWithCovarianceHistory::subscribe()
{
  MFDClass::subscribe();
}

void PoseWithCovarianceHistory::unsubscribe()
{
  MFDClass::unsubscribe();

  history_.clear();
  lines_->clear();
  spheres_.clear();
}

void PoseWithCovarianceHistory::processMessage(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
{
  if (!rviz_common::validateFloats(message->pose.pose) || !rviz_common::validateFloats(message->pose.covariance)) {
    setStatus(
      rviz_common::properties::StatusProperty::Error, "Topic",
      "Message contained invalid floating point values (nans or infs)");
    return;
  }
  if (target_frame_ != message->header.frame_id) {
    history_.clear();
    spheres_.clear();
    target_frame_ = message->header.frame_id;
  }
  history_.emplace_back(message);
  last_stamp_ = message->header.stamp;
  updateHistory();
}

void PoseWithCovarianceHistory::updateHistory()
{
  const auto buffer_size = static_cast<size_t>(property_buffer_size_->getInt());
  while (buffer_size < history_.size()) {
    history_.pop_front();
  }
}

void PoseWithCovarianceHistory::updateShapes()
{
   int shape_type = property_shape_type_->getOptionInt();
  Ogre::ColourValue color_line = rviz_common::properties::qtToOgre(property_line_color_->getColor());
  color_line.a = property_line_alpha_->getFloat();
  Ogre::ColourValue color_shape = rviz_common::properties::qtToOgre(property_shape_color_->getColor());
  color_shape.a = property_shape_alpha_->getFloat();
  Ogre::ColourValue color_arrow = rviz_common::properties::qtToOgre(property_arrow_color_->getColor());
  color_arrow.a = property_arrow_alpha_->getFloat();

  Ogre::Vector3 line_position;
  Ogre::Quaternion line_orientation;

  auto frame_manager = context_->getFrameManager();
  if (!frame_manager->getTransform(target_frame_, last_stamp_, line_position, line_orientation)) {
    setMissingTransformToFixedFrame(target_frame_);
    return;
  }

  setTransformOk();
  lines_->setMaxPointsPerLine(history_.size());
  lines_->setLineWidth(property_line_width_->getFloat());
  lines_->setPosition(line_position);
  lines_->setOrientation(line_orientation);
  lines_->setColor(color_line.r, color_line.g, color_line.b, color_line.a);

  while (spheres_.size() < history_.size()) {
    spheres_.emplace_back(std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_));
  }
  while (arrows_.size() < history_.size()) {
    arrows_.emplace_back(std::make_unique<rviz_rendering::Arrow>(scene_manager_, scene_node_));
  }

  for (size_t i = 0; i < history_.size(); ++i) {
    const auto& message = history_[i];
   
    Ogre::Vector3 position;
    position.x = message->pose.pose.position.x;
    position.y = message->pose.pose.position.y;
    position.z = message->pose.pose.position.z;
    if (shape_type == 0) {
      lines_->addPoint(position);
    }

    Ogre::Quaternion orientation;
    orientation.w = message->pose.pose.orientation.w;
    orientation.x = message->pose.pose.orientation.x;
    orientation.y = message->pose.pose.orientation.y;
    orientation.z = message->pose.pose.orientation.z;
    
    Eigen::Matrix2d covariance_2d_map;
    covariance_2d_map(0, 0) = message->pose.covariance[0];
    covariance_2d_map(1, 1) = message->pose.covariance[1 + 6 * 1];
    covariance_2d_map(1, 0) = message->pose.covariance[1 + 6 * 0];
    covariance_2d_map(0, 1) = message->pose.covariance[0 + 6 * 1];

    Eigen::Matrix2d covariance_2d_base_link;
    Eigen::Translation3f translation(message->pose.pose.position.x, message->pose.pose.position.y, message->pose.pose.position.z);
    Eigen::Quaternionf rotation(message->pose.pose.orientation.w, message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z);
    Eigen::Matrix4f pose_matrix4f = (translation * rotation).matrix();
    const Eigen::Matrix2d rot = pose_matrix4f.topLeftCorner<2, 2>().cast<double>();
    covariance_2d_base_link = rot.transpose() * covariance_2d_map * rot;//対角化

    auto& sphere = spheres_[i];
    sphere->setPosition(position);
    sphere->setOrientation(orientation);
    sphere->setColor(color_shape.r, color_shape.g, color_shape.b, color_shape.a);
    sphere->setScale(Ogre::Vector3(
      property_shape_scale_->getFloat() * 2 * std::sqrt(covariance_2d_base_link(0, 0)),
      property_shape_scale_->getFloat() * 2 * std::sqrt(covariance_2d_base_link(1, 1)),
      property_shape_scale_->getFloat() * 2 * std::sqrt(message->pose.covariance[14])));
    
    if (shape_type == 1) {
      auto& arrow = arrows_[i];
      arrow->set(property_arrow_shaft_length_->getFloat(), property_arrow_shaft_diameter_->getFloat(), 
        property_arrow_head_length_->getFloat(), property_arrow_head_diameter_->getFloat());
      arrow->setPosition(position);
      Ogre::Quaternion y90(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y);
      arrow->setOrientation(orientation * y90);
      arrow->setColor(color_arrow.r, color_arrow.g, color_arrow.b, color_arrow.a);
    }
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseWithCovarianceHistory, rviz_common::Display)
