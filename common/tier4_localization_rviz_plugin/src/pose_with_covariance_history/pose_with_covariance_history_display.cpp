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
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_rendering/objects/shape.hpp>

namespace rviz_plugins
{
PoseWithCovarianceHistory::PoseWithCovarianceHistory() : last_stamp_(0, 0, RCL_ROS_TIME)
{
  property_buffer_size_ = new rviz_common::properties::IntProperty("Buffer Size", 100, "", this);
  property_shape_view_ = new rviz_common::properties::BoolProperty("Shape", true, "", this);
  property_shape_alpha_ =
    new rviz_common::properties::FloatProperty("Alpha", 1.0, "", property_shape_view_);
  property_shape_alpha_->setMin(0.0);
  property_shape_alpha_->setMax(1.0);
  property_shape_color_ =
    new rviz_common::properties::ColorProperty("Color", Qt::white, "", property_shape_view_);
  property_shape_scale_ =
    new rviz_common::properties::FloatProperty("Scale", 1.0, "", property_shape_view_);
  property_shape_scale_->setMin(0.0);
  property_shape_scale_->setMax(1000);

  property_buffer_size_->setMin(0);
  property_buffer_size_->setMax(16000);  
}

PoseWithCovarianceHistory::~PoseWithCovarianceHistory() = default;  // Properties are deleted by Qt

void PoseWithCovarianceHistory::onInitialize()
{
  MFDClass::onInitialize();
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
    shapes_.clear();
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
  shapes_.clear();
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
    shapes_.clear();
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
  Ogre::ColourValue color = rviz_common::properties::qtToOgre(property_shape_color_->getColor());
  color.a = property_shape_alpha_->getFloat();
  // Ogre::Vector3 position;
  // Ogre::Quaternion orientation;

  // auto frame_manager = context_->getFrameManager();
  // if (!frame_manager->getTransform(target_frame_, last_stamp_, position, orientation)) {
  //   setMissingTransformToFixedFrame(target_frame_);
  //   return;
  // }

  // setTransformOk();
   while (shapes_.size() < history_.size()) {
    shapes_.emplace_back(std::make_unique<rviz_rendering::Shape>(rviz_rendering::Shape::Sphere, scene_manager_, scene_node_));
  }

  for (size_t i = 0; i < history_.size(); ++i) {
    const auto& message = history_[i];
   
    Ogre::Vector3 position;//ok
    position.x = message->pose.pose.position.x;
    position.y = message->pose.pose.position.y;
    position.z = message->pose.pose.position.z;
    
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
    covariance_2d_base_link = rot.transpose() * covariance_2d_map * rot;

    Ogre::Quaternion orientation;
    orientation.w = message->pose.pose.orientation.w;
    orientation.x = message->pose.pose.orientation.x;
    orientation.y = message->pose.pose.orientation.y;
    orientation.z = message->pose.pose.orientation.z;

    //covariance をbase_link座標系にしてx,y,zを取り出す
    auto& sphere = shapes_[i];
    sphere->setPosition(position);
    sphere->setOrientation(orientation);
    sphere->setColor(color.r, color.g, color.b, color.a);
    sphere->setScale(Ogre::Vector3(
      property_shape_scale_->getFloat() * 2 * std::sqrt(covariance_2d_base_link(0, 0)),
      property_shape_scale_->getFloat() * 2 * std::sqrt(covariance_2d_base_link(1, 1)),
      property_shape_scale_->getFloat() * 2 * std::sqrt(message->pose.covariance[14])));
  }
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PoseWithCovarianceHistory, rviz_common::Display)
