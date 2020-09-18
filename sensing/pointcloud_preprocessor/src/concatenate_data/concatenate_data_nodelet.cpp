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

#include "pointcloud_preprocessor/concatenate_data/concatenate_data_nodelet.h"

#include <pcl_ros/transforms.h>
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>

//////////////////////////////////////////////////////////////////////////////////////////////

namespace pointcloud_preprocessor
{
void PointCloudConcatenateDataSynchronizerNodelet::onInit()
{
  nodelet_topic_tools::NodeletLazy::onInit();

  // ---[ Mandatory parameters
  pnh_->getParam("output_frame", output_frame_);

  if (output_frame_.empty()) {
    NODELET_ERROR("[onInit] Need an 'output_frame' parameter to be set before continuing!");
    return;
  }

  if (!pnh_->getParam("input_topics", input_topics_)) {
    NODELET_ERROR("[onInit] Need a 'input_topics' parameter to be set before continuing!");
    return;
  }
  if (input_topics_.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    NODELET_ERROR("[onInit] Invalid 'input_topics' parameter given!");
    return;
  }
  if (input_topics_.size() == 1) {
    NODELET_ERROR("[onInit] Only one topic given. Need at least two topics to continue.");
    return;
  }

  // ---[ Optional parameters
  pnh_->getParam("max_queue_size", maximum_queue_size_);
  pnh_->getParam("timeout_sec", timeout_sec_);

  // Output
  pub_output_ = advertise<PointCloud2>(*pnh_, "output", maximum_queue_size_);
  pub_concat_num_ = advertise<std_msgs::Int32>(*pnh_, "concat_num", 10);
  pub_not_subscribed_topic_name_ =
    advertise<std_msgs::String>(*pnh_, "not_subscribed_topic_name", 10);

  onInitPostProcess();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerNodelet::subscribe()
{
  ROS_INFO_STREAM("Subscribing to " << input_topics_.size() << " user given topics as inputs:");
  for (int d = 0; d < input_topics_.size(); ++d)
    ROS_INFO_STREAM(" - " << (std::string)(input_topics_[d]));

  // Subscribe to the filters
  filters_.resize(input_topics_.size());

  // First input_topics_.size () filters are valid
  for (int d = 0; d < input_topics_.size(); ++d) {
    cloud_stdmap_.insert(std::make_pair((std::string)(input_topics_[d]), nullptr));
    cloud_stdmap_tmp_ = cloud_stdmap_;

    filters_[d].reset(new ros::Subscriber());
    *filters_[d] = pnh_->subscribe<sensor_msgs::PointCloud2>(
      (std::string)(input_topics_[d]), maximum_queue_size_,
      bind(
        &PointCloudConcatenateDataSynchronizerNodelet::cloud_callback, this, _1,
        (std::string)(input_topics_[d])));
  }

  sub_twist_ = pnh_->subscribe<geometry_msgs::TwistStamped>(
    "/vehicle/status/twist", 100,
    bind(&PointCloudConcatenateDataSynchronizerNodelet::twist_callback, this, _1));
  timer_ = pnh_->createTimer(
    ros::Duration(timeout_sec_), &PointCloudConcatenateDataSynchronizerNodelet::timer_callback,
    this, true);
  timer_.stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerNodelet::unsubscribe()
{
  for (size_t d = 0; d < filters_.size(); ++d) {
    filters_[d]->shutdown();
  }
  sub_twist_.shutdown();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudConcatenateDataSynchronizerNodelet::transformPointCloud(
  const PointCloud2::ConstPtr & in, PointCloud2::Ptr & out)
{
  // Transform the point clouds into the specified output frame
  if (output_frame_ != in->header.frame_id) {
    // TODO use TF2
    if (!pcl_ros::transformPointCloud(output_frame_, *in, *out, tf_)) {
      NODELET_ERROR(
        "[%s::transformPointCloud] Error converting first input dataset from %s to %s.",
        getName().c_str(), in->header.frame_id.c_str(), output_frame_.c_str());
      return;
    }
  } else {
    out = boost::make_shared<PointCloud2>(*in);
  }
}

void PointCloudConcatenateDataSynchronizerNodelet::combineClouds(
  const PointCloud2::ConstPtr & in1, const PointCloud2::ConstPtr & in2, PointCloud2::Ptr & out)
{
  if (twist_ptr_queue_.empty()) {
    pcl::concatenatePointCloud(*in1, *in2, *out);
    out->header.stamp = std::min(in1->header.stamp, in2->header.stamp);
    return;
  }

  const auto old_stamp = std::min(in1->header.stamp, in2->header.stamp);
  auto old_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), old_stamp,
    [](const geometry_msgs::TwistStamped::ConstPtr & x_ptr, ros::Time t) {
      return x_ptr->header.stamp < t;
    });
  old_twist_ptr_it =
    old_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : old_twist_ptr_it;

  const auto new_stamp = std::max(in1->header.stamp, in2->header.stamp);
  auto new_twist_ptr_it = std::lower_bound(
    std::begin(twist_ptr_queue_), std::end(twist_ptr_queue_), new_stamp,
    [](const geometry_msgs::TwistStamped::ConstPtr & x_ptr, ros::Time t) {
      return x_ptr->header.stamp < t;
    });
  new_twist_ptr_it =
    new_twist_ptr_it == twist_ptr_queue_.end() ? (twist_ptr_queue_.end() - 1) : new_twist_ptr_it;

  auto prev_time = old_stamp;
  double x = 0.0, y = 0.0, yaw = 0.0;
  for (auto twist_ptr_it = old_twist_ptr_it; twist_ptr_it != new_twist_ptr_it + 1; ++twist_ptr_it) {
    const double dt = (twist_ptr_it != new_twist_ptr_it)
                        ? ((*twist_ptr_it)->header.stamp - prev_time).toSec()
                        : (new_stamp - prev_time).toSec();

    if (std::fabs(dt) > 0.1) {
      ROS_WARN_STREAM_THROTTLE(
        10,
        "Time difference is too large. Cloud not interpolate. Please comfirm twist topic and "
        "timestamp");
      break;
    }

    const double dis = (*twist_ptr_it)->twist.linear.x * dt;
    yaw += (*twist_ptr_it)->twist.angular.z * dt;
    x += dis * std::cos(yaw);
    y += dis * std::sin(yaw);
    prev_time = (*twist_ptr_it)->header.stamp;
  }
  Eigen::AngleAxisf rotation_x(0, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(0, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f translation(x, y, 0);
  Eigen::Matrix4f rotation_matrix = (translation * rotation_z * rotation_y * rotation_x).matrix();

  // TODO if output_frame_ is not base_link, we must transform

  if (in1->header.stamp > in2->header.stamp) {
    sensor_msgs::PointCloud2::Ptr in1_t(new sensor_msgs::PointCloud2());
    pcl_ros::transformPointCloud(rotation_matrix, *in1, *in1_t);
    pcl::concatenatePointCloud(*in1_t, *in2, *out);
    out->header.stamp = in2->header.stamp;
  } else {
    sensor_msgs::PointCloud2::Ptr in2_t(new sensor_msgs::PointCloud2());
    pcl_ros::transformPointCloud(rotation_matrix, *in2, *in2_t);
    pcl::concatenatePointCloud(*in1, *in2_t, *out);
    out->header.stamp = in1->header.stamp;
  }
}

void PointCloudConcatenateDataSynchronizerNodelet::publish()
{
  sensor_msgs::PointCloud2::Ptr concat_cloud_ptr_ = nullptr;
  std::string not_subscribed_topic_name = "";
  size_t concat_num = 0;

  for (const auto & e : cloud_stdmap_) {
    if (e.second != nullptr) {
      sensor_msgs::PointCloud2::Ptr transed_cloud_ptr(new sensor_msgs::PointCloud2());
      transformPointCloud(e.second, transed_cloud_ptr);
      if (concat_cloud_ptr_ == nullptr) {
        concat_cloud_ptr_ = transed_cloud_ptr;
      } else {
        PointCloudConcatenateDataSynchronizerNodelet::combineClouds(
          concat_cloud_ptr_, transed_cloud_ptr, concat_cloud_ptr_);
      }
      ++concat_num;

    } else {
      if (not_subscribed_topic_name.empty()) {
        not_subscribed_topic_name = e.first;
      } else {
        not_subscribed_topic_name += "," + e.first;
      }
    }
  }
  if (!not_subscribed_topic_name.empty()) {
    ROS_WARN_STREAM_THROTTLE(
      1, "Skipped " << not_subscribed_topic_name << ". Please confirm topic.");
  }

  pub_output_.publish(*concat_cloud_ptr_);

  std_msgs::Int32 concat_num_msg;
  concat_num_msg.data = concat_num;
  pub_concat_num_.publish(concat_num_msg);

  std_msgs::String not_subscribed_topic_name_msg;
  not_subscribed_topic_name_msg.data = not_subscribed_topic_name;
  pub_not_subscribed_topic_name_.publish(not_subscribed_topic_name_msg);

  cloud_stdmap_ = cloud_stdmap_tmp_;
  std::for_each(std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_), [](auto & e) {
    e.second = nullptr;
  });
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudConcatenateDataSynchronizerNodelet::cloud_callback(
  const sensor_msgs::PointCloud2::ConstPtr & input_ptr, const std::string & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  const bool is_already_subscribed_this = (cloud_stdmap_[topic_name] != nullptr);
  const bool is_already_subscribed_tmp = std::any_of(
    std::begin(cloud_stdmap_tmp_), std::end(cloud_stdmap_tmp_),
    [](const auto & e) { return e.second != nullptr; });

  static ros::Time timer_start_time;
  if (is_already_subscribed_this) {
    cloud_stdmap_tmp_[topic_name] = input_ptr;

    if (!is_already_subscribed_tmp) {
      // ROS_INFO("Subscribed already received topic. Create Timer");
      timer_.setPeriod(ros::Duration(timeout_sec_), true);
      timer_.start();
      timer_start_time = ros::Time::now();
    }
  } else {
    if (is_already_subscribed_tmp) {
      cloud_stdmap_tmp_[topic_name] = input_ptr;
      // std::cout << (ros::Time::now() - timer_start_time).toSec() << std::endl;
    }

    cloud_stdmap_[topic_name] = input_ptr;
    const bool is_subscribed_all = std::all_of(
      std::begin(cloud_stdmap_), std::end(cloud_stdmap_),
      [](const auto & e) { return e.second != nullptr; });

    if (is_subscribed_all) {
      for (const auto & e : cloud_stdmap_tmp_) {
        if (e.second != nullptr) {
          cloud_stdmap_[e.first] = e.second;
        }
      }

      timer_.stop();
      publish();
    }
  }
}

void PointCloudConcatenateDataSynchronizerNodelet::timer_callback(const ros::TimerEvent &)
{
  // ROS_WARN("TimeOut. Please confirm PointCloud topic");

  std::lock_guard<std::mutex> lock(mutex_);
  timer_.stop();
  publish();
}

void PointCloudConcatenateDataSynchronizerNodelet::twist_callback(
  const geometry_msgs::TwistStamped::ConstPtr & input)
{
  // if rosbag restart, clear buffer
  if (!twist_ptr_queue_.empty()) {
    if (twist_ptr_queue_.front()->header.stamp > input->header.stamp) {
      twist_ptr_queue_.clear();
    }
  }

  // pop old data
  while (!twist_ptr_queue_.empty()) {
    if (twist_ptr_queue_.front()->header.stamp + ros::Duration(1.0) > input->header.stamp) {
      break;
    }
    twist_ptr_queue_.pop_front();
  }
  twist_ptr_queue_.push_back(input);
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(
  pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerNodelet, nodelet::Nodelet);
