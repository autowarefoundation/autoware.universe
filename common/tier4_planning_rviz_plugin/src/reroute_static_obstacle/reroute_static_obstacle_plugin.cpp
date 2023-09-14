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
/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <reroute_static_obstacle/reroute_static_obstacle_plugin.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>

#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgreViewport.h>

#include <geometry_msgs/msg/point_stamped.hpp>

namespace rviz_plugins
{

RerouteStaticObstaclePointPublish::RerouteStaticObstaclePointPublish() : qos_profile_(5)
{
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/simulation/planning/reroute_static_obstacle_point_publisher/point",
    "The topic on which to publish points.", getPropertyContainer(), SLOT(updateTopic()), this);

  auto_deactivate_property_ = new rviz_common::properties::BoolProperty(
    "Single click", true, "Switch away from this tool after one click.", getPropertyContainer(),
    SLOT(updateAutoDeactivate()), this);

  qos_profile_property_ =
    new rviz_common::properties::QosProfileProperty(topic_property_, qos_profile_);
}

void RerouteStaticObstaclePointPublish::onInitialize()
{
  qos_profile_property_->initialize([this](rclcpp::QoS profile) { this->qos_profile_ = profile; });
  updateTopic();
}

void RerouteStaticObstaclePointPublish::activate()
{
}

void RerouteStaticObstaclePointPublish::deactivate()
{
}

void RerouteStaticObstaclePointPublish::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  publisher_ = raw_node->template create_publisher<geometry_msgs::msg::PointStamped>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void RerouteStaticObstaclePointPublish::updateAutoDeactivate()
{
}

int RerouteStaticObstaclePointPublish::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;
  if (event.leftUp()) {
    const auto point = get_point_from_mouse(event);
    if (point) {
      setStatusForPosition(point.value());
      publishPosition(point.value());
      if (auto_deactivate_property_->getBool()) {
        flags |= Finished;
      }
    }
  }

  return flags;
}

void RerouteStaticObstaclePointPublish::setStatusForPosition(const Ogre::Vector3 & position)
{
  std::ostringstream s;
  s << "<b>Left-Click:</b> Select this point.";
  s.precision(3);
  s << " [" << position.x << "," << position.y << "," << position.z << "]";
  setStatus(s.str().c_str());
}

void RerouteStaticObstaclePointPublish::publishPosition(const Ogre::Vector3 & position) const
{
  auto point = rviz_common::pointOgreToMsg(position);
  geometry_msgs::msg::PointStamped point_stamped;
  point_stamped.point = point;
  point_stamped.header.frame_id = context_->getFixedFrame().toStdString();
  point_stamped.header.stamp = clock_->now();
  publisher_->publish(point_stamped);
}

std::optional<Ogre::Vector3> RerouteStaticObstaclePointPublish::get_point_from_mouse(
  rviz_common::ViewportMouseEvent & event)
{
  using rviz_rendering::RenderWindowOgreAdapter;
  const auto viewport = RenderWindowOgreAdapter::getOgreViewport(event.panel->getRenderWindow());
  const auto w = viewport->getActualWidth();
  const auto h = viewport->getActualHeight();
  const auto x = static_cast<Ogre::Real>(event.x) / static_cast<Ogre::Real>(w);
  const auto y = static_cast<Ogre::Real>(event.y) / static_cast<Ogre::Real>(h);

  const auto plane = Ogre::Plane(Ogre::Vector3::UNIT_Z, 0.0);
  const auto ray = viewport->getCamera()->getCameraToViewportRay(x, y);
  const auto intersect = ray.intersects(plane);
  return intersect.first ? std::optional(ray.getPoint(intersect.second)) : std::nullopt;
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_plugins::RerouteStaticObstaclePointPublish, rviz_common::Tool)
