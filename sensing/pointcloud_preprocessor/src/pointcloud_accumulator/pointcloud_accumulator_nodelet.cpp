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

#include "pointcloud_preprocessor/pointcloud_accumulator/pointcloud_accumulator_nodelet.h"
#include <pluginlib/class_list_macros.h>

namespace pointcloud_preprocessor
{
bool PointcloudAccumulatorNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::PointcloudAccumulatorConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::PointcloudAccumulatorConfig>::CallbackType
    f = boost::bind(&PointcloudAccumulatorNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void PointcloudAccumulatorNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pointcloud_buffer_.push_front(input);
  ros::Time last_time = input->header.stamp;
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  for (size_t i = 0; i < pointcloud_buffer_.size(); i++) {
    if (accumulation_time_sec_ < (last_time - pointcloud_buffer_.at(i)->header.stamp).toSec())
      break;
    pcl::fromROSMsg(*pointcloud_buffer_.at(i), pcl_input);
    pcl_output += pcl_input;
  }
  pcl::toROSMsg(pcl_output, output);
  output.header = input->header;
}

void PointcloudAccumulatorNodelet::subscribe() { Filter::subscribe(); }

void PointcloudAccumulatorNodelet::unsubscribe() { Filter::unsubscribe(); }

void PointcloudAccumulatorNodelet::config_callback(
  pointcloud_preprocessor::PointcloudAccumulatorConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (accumulation_time_sec_ != config.accumulation_time_sec) {
    accumulation_time_sec_ = config.accumulation_time_sec;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new accumulation time to: %f.", getName().c_str(),
      config.accumulation_time_sec);
  }
  if (pointcloud_buffer_.size() != (size_t)config.pointcloud_buffer_size) {
    NODELET_DEBUG(
      "[%s::config_callback] Setting new buffer size to: %d.", getName().c_str(),
      config.pointcloud_buffer_size);
  }
  pointcloud_buffer_.set_capacity((size_t)config.pointcloud_buffer_size);
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit
  // from Filter
  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG("[config_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG(
      "[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::PointcloudAccumulatorNodelet, nodelet::Nodelet);