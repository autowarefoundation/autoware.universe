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
#pragma once

#include <deque>
#include <map>
#include <mutex>

// ROS includes
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <nodelet_topic_tools/nodelet_lazy.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace pointcloud_preprocessor
{
/** \brief @b PointCloudConcatenateFieldsSynchronizer is a special form of data
 * synchronizer: it listens for a set of input PointCloud messages on the same topic,
 * checks their timestamps, and concatenates their fields together into a single
 * PointCloud output message.
 * \author Radu Bogdan Rusu
 */
class PointCloudConcatenateDataSynchronizerNodelet : public nodelet_topic_tools::NodeletLazy
{
public:
  typedef sensor_msgs::PointCloud2 PointCloud2;
  typedef PointCloud2::Ptr PointCloud2Ptr;
  typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

  /** \brief Empty constructor. */
  PointCloudConcatenateDataSynchronizerNodelet() : maximum_queue_size_(3), timeout_sec_(0.03){};

  /** \brief Empty constructor.
   * \param queue_size the maximum queue size
   */
  PointCloudConcatenateDataSynchronizerNodelet(int queue_size)
  : maximum_queue_size_(queue_size), timeout_sec_(0.03){};

  /** \brief Empty destructor. */
  virtual ~PointCloudConcatenateDataSynchronizerNodelet(){};

  void onInit();
  void subscribe();
  void unsubscribe();

private:
  /** \brief The output PointCloud publisher. */
  ros::Publisher pub_output_;

  ros::Publisher pub_concat_num_;
  ros::Publisher pub_not_subscribed_topic_name_;

  /** \brief The maximum number of messages that we can store in the queue. */
  int maximum_queue_size_;

  double timeout_sec_;

  /** \brief A vector of subscriber. */
  std::vector<boost::shared_ptr<ros::Subscriber> > filters_;

  ros::Subscriber sub_twist_;

  ros::Timer timer_;

  /** \brief Output TF frame the concatenated points should be transformed to. */
  std::string output_frame_;

  /** \brief Input point cloud topics. */
  XmlRpc::XmlRpcValue input_topics_;

  /** \brief TF listener object. */
  tf::TransformListener tf_;

  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_ptr_queue_;

  std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_;
  std::map<std::string, sensor_msgs::PointCloud2::ConstPtr> cloud_stdmap_tmp_;
  std::mutex mutex_;

  void transformPointCloud(const PointCloud2::ConstPtr & in, PointCloud2::Ptr & out);
  void combineClouds(
    const PointCloud2::ConstPtr & in1, const PointCloud2::ConstPtr & in2, PointCloud2::Ptr & out);
  void publish();

  void convertToXYZCloud(
    const sensor_msgs::PointCloud2 & input_cloud,
    sensor_msgs::PointCloud2 & output_cloud);
  void cloud_callback(
    const sensor_msgs::PointCloud2::ConstPtr & input_ptr, const std::string & topic_name);
  void twist_callback(const geometry_msgs::TwistStamped::ConstPtr & input);
  void timer_callback(const ros::TimerEvent &);
};
}  // namespace pointcloud_preprocessor
