/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * 
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: cropbox.cpp 
 *
 */

#include "pointcloud_preprocessor/crop_box_filter/crop_box_filter_nodelet.h"

#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/PolygonStamped.h>

namespace pointcloud_preprocessor
{
bool CropBoxFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ =
    boost::make_shared<dynamic_reconfigure::Server<pointcloud_preprocessor::CropBoxFilterConfig> >(
      nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::CropBoxFilterConfig>::CallbackType f =
    boost::bind(&CropBoxFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);

  crop_box_polygon_pub_ = advertise<geometry_msgs::PolygonStamped>(*pnh_, "crop_box_polygon", 10);
  return (true);
}

void CropBoxFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PCLPointCloud2::Ptr pcl_input(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*(input), *(pcl_input));
  impl_.setInputCloud(pcl_input);
  impl_.setIndices(indices);
  pcl::PCLPointCloud2 pcl_output;
  impl_.filter(pcl_output);
  pcl_conversions::moveFromPCL(pcl_output, output);

  publishCropBoxPolygon();
}

void CropBoxFilterNodelet::publishCropBoxPolygon()
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::Point32 point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  const double x1 = impl_.getMax()(0);
  const double x2 = impl_.getMin()(0);
  const double x3 = impl_.getMin()(0);
  const double x4 = impl_.getMax()(0);

  const double y1 = impl_.getMax()(1);
  const double y2 = impl_.getMax()(1);
  const double y3 = impl_.getMin()(1);
  const double y4 = impl_.getMin()(1);

  const double z1 = impl_.getMin()(2);
  const double z2 = impl_.getMax()(2);

  geometry_msgs::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = tf_input_frame_;
  polygon_msg.header.stamp = ros::Time::now();
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z1));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x2, y2, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x3, y3, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z1));
  polygon_msg.polygon.points.push_back(generatePoint(x4, y4, z2));

  polygon_msg.polygon.points.push_back(generatePoint(x1, y1, z2));

  crop_box_polygon_pub_.publish(polygon_msg);
}

void CropBoxFilterNodelet::subscribe() { Filter::subscribe(); }

void CropBoxFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void CropBoxFilterNodelet::config_callback(
  pointcloud_preprocessor::CropBoxFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  Eigen::Vector4f min_point, max_point;
  min_point = impl_.getMin();
  max_point = impl_.getMax();

  Eigen::Vector4f new_min_point, new_max_point;
  new_min_point << config.min_x, config.min_y, config.min_z, 0.0;
  new_max_point << config.max_x, config.max_y, config.max_z, 0.0;

  // Check the current values for minimum point
  if (min_point != new_min_point) {
    NODELET_DEBUG(
      "[%s::config_callback] Setting the minimum point to: %f %f %f.", getName().c_str(),
      new_min_point(0), new_min_point(1), new_min_point(2));
    // Set the filter min point if different
    impl_.setMin(new_min_point);
  }
  // Check the current values for the maximum point
  if (max_point != new_max_point) {
    NODELET_DEBUG(
      "[%s::config_callback] Setting the maximum point to: %f %f %f.", getName().c_str(),
      new_max_point(0), new_max_point(1), new_max_point(2));
    // Set the filter max point if different
    impl_.setMax(new_max_point);
  }

  // Check the current value for keep_organized
  if (impl_.getKeepOrganized() != config.keep_organized) {
    NODELET_DEBUG(
      "[%s::config_callback] Setting the filter keep_organized value to: %s.", getName().c_str(),
      config.keep_organized ? "true" : "false");
    // Call the virtual method in the child
    impl_.setKeepOrganized(config.keep_organized);
  }

  // Check the current value for the negative flag
  if (impl_.getNegative() != config.negative) {
    NODELET_DEBUG(
      "[%s::config_callback] Setting the filter negative flag to: %s.", getName().c_str(),
      config.negative ? "true" : "false");
    // Call the virtual method in the child
    impl_.setNegative(config.negative);
  }

  // The following parameters are updated automatically for all PCL_ROS Nodelet Filters as they are inexistent in PCL
  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG(
      "[%s::config_callback] Setting the input TF frame to: %s.", getName().c_str(),
      tf_input_frame_.c_str());
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG(
      "[%s::config_callback] Setting the output TF frame to: %s.", getName().c_str(),
      tf_output_frame_.c_str());
  }
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::CropBoxFilterNodelet, nodelet::Nodelet);