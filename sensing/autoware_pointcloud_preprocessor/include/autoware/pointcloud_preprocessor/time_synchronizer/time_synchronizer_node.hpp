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
 * $Id: TIME_SYNCHRONIZER.cpp 35231 2011-01-14 05:33:20Z rusu $
 *
 */

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__TIME_SYNCHRONIZER__TIME_SYNCHRONIZER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__TIME_SYNCHRONIZER__TIME_SYNCHRONIZER_NODE_HPP_

#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

// ROS includes
#include <autoware/point_types/types.hpp>
#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/managed_transform_buffer.hpp>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>

#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/string_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace autoware::pointcloud_preprocessor
{
using autoware::point_types::PointXYZIRC;
using point_cloud_msg_wrapper::PointCloud2Modifier;
// cspell:ignore Yoshi
/** \brief @b PointCloudDataSynchronizerComponent is a special form of data
 * synchronizer: it listens for a set of input PointCloud messages on the same topic,
 * checks their timestamps, and concatenates their fields together into a single
 * PointCloud output message.
 * \author Radu Bogdan Rusu
 * \edited by Yoshi Ri
 */
class PointCloudDataSynchronizerComponent : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  /** \brief constructor. */
  explicit PointCloudDataSynchronizerComponent(const rclcpp::NodeOptions & node_options);

  /** \brief constructor.
   * \param queue_size the maximum queue size
   */
  PointCloudDataSynchronizerComponent(const rclcpp::NodeOptions & node_options, int queue_size);

  /** \brief Empty destructor. */
  virtual ~PointCloudDataSynchronizerComponent() {}

private:
  /** \brief Delay Compensated PointCloud publisher*/
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
    transformed_raw_pc_publisher_map_;

  /** \brief The maximum number of messages that we can store in the queue. */
  int maximum_queue_size_ = 3;

  double timeout_sec_ = 0.1;

  std::set<std::string> not_subscribed_topic_names_;

  /** \brief A vector of subscriber. */
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> filters_;

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::TimerBase::SharedPtr timer_;
  diagnostic_updater::Updater updater_{this};

  const std::string input_twist_topic_type_;

  /** \brief Output TF frame the concatenated points should be transformed to. */
  std::string output_frame_;
  bool keep_input_frame_in_synchronized_pointcloud_;

  /** \brief The flag to indicate if only static TF are used. */
  bool has_static_tf_only_;

  /** \brief Input point cloud topics. */
  // XmlRpc::XmlRpcValue input_topics_;
  std::vector<std::string> input_topics_;

  std::unique_ptr<autoware::universe_utils::ManagedTransformBuffer> managed_tf_buffer_{nullptr};

  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_ptr_queue_;

  std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_;
  std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> cloud_stdmap_tmp_;
  std::mutex mutex_;

  std::vector<double> input_offset_;
  std::map<std::string, double> offset_map_;

  Eigen::Matrix4f computeTransformToAdjustForOldTimestamp(
    const rclcpp::Time & old_stamp, const rclcpp::Time & new_stamp);
  std::map<std::string, sensor_msgs::msg::PointCloud2::SharedPtr> synchronizeClouds();
  void publish();

  void convertToXYZIRCCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr & input_ptr,
    sensor_msgs::msg::PointCloud2::SharedPtr & output_ptr);
  void setPeriod(const int64_t new_period);
  void cloud_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_ptr,
    const std::string & topic_name);
  void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr input);
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr input);
  void timer_callback();

  void checkSyncStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::string replaceSyncTopicNamePostfix(
    const std::string & original_topic_name, const std::string & postfix);

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware::universe_utils::DebugPublisher> debug_publisher_;
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__TIME_SYNCHRONIZER__TIME_SYNCHRONIZER_NODE_HPP_
