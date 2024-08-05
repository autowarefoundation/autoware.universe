// Copyright 2024 TIER IV, Inc.
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
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: concatenate_data.cpp 35231 2011-01-14 05:33:20Z rusu $
 *
 */

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_COLLECTOR_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_COLLECTOR_HPP_

#include "combine_cloud_handler.hpp"

#include <list>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

class PointCloudConcatenateDataSynchronizerComponent;
class CombineCloudHandler;

class CloudCollector
{
public:
  CloudCollector(
    std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> concatenate_node,
    std::list<std::shared_ptr<CloudCollector>> & collectors,
    std::shared_ptr<CombineCloudHandler> combine_cloud_handler, int num_of_clouds, double time);

  void setReferenceTimeStamp(double timestamp, double noise_window);
  std::tuple<double, double> getReferenceTimeStampBoundary();
  void processCloud(std::string topic_name, sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  void concatenateCallback();
  std::tuple<
    sensor_msgs::msg::PointCloud2::SharedPtr,
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>,
    std::unordered_map<std::string, double>>
  concatenateClouds(
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map);

  void publishClouds(
    sensor_msgs::msg::PointCloud2::SharedPtr concatenate_cloud_ptr,
    std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr>
      topic_to_transformed_cloud_map,
    std::unordered_map<std::string, double> topic_to_original_stamp_map);

  void deleteCollector();

  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> getTopicToCloudMap();

private:
  std::shared_ptr<PointCloudConcatenateDataSynchronizerComponent> concatenate_node_;
  std::list<std::shared_ptr<CloudCollector>> & collectors_;
  std::shared_ptr<CombineCloudHandler> combine_cloud_handler_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> topic_to_cloud_map_;
  uint64_t num_of_clouds_;
  double timeout_sec_;
  double reference_timestamp_min_;
  double reference_timestamp_max_;
  std::mutex mutex_;
};

}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__CONCATENATE_DATA__CLOUD_COLLECTOR_HPP_  // NOLINT
// clang-format on
