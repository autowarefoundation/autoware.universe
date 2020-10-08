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
 * $Id: passthrough.cpp 36194 2011-02-23 07:49:21Z rusu $
 *
 */

#include "pointcloud_preprocessor/passthrough_filter/passthrough_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
bool PassThroughFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::PassThroughFilterConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::PassThroughFilterConfig>::CallbackType f =
    boost::bind(&PassThroughFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void PassThroughFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  output = *input;
}

void PassThroughFilterNodelet::subscribe() { Filter::subscribe(); }

void PassThroughFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void PassThroughFilterNodelet::config_callback(
  pointcloud_preprocessor::PassThroughFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

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

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::PassThroughFilterNodelet, nodelet::Nodelet);