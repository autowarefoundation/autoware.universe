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

#ifndef YABLOC_IMAGE_PROCESSING__SEGMENT_FILTER__SEGMENT_FILTER_HPP_
#define YABLOC_IMAGE_PROCESSING__SEGMENT_FILTER__SEGMENT_FILTER_HPP_

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yabloc_common/camera_info_subscriber.hpp>
#include <yabloc_common/static_tf_subscriber.hpp>
#include <yabloc_common/synchro_subscriber.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <set>

namespace yabloc::segment_filter
{
class SegmentFilter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;

  SegmentFilter();

private:
  const int image_size_;

  common::SynchroSubscriber<PointCloud2, Image> synchro_subscriber_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_classified_cloud_;

  std::set<int> filter_by_mask(
    const cv::Mat & mask, const pcl::PointCloud<pcl::PointNormal> & edges);

  void execute(const PointCloud2 & msg1, const Image & msg2);
};
}  // namespace yabloc::segment_filter

#endif  // YABLOC_IMAGE_PROCESSING__SEGMENT_FILTER__SEGMENT_FILTER_HPP_
