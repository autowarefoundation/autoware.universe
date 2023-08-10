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

#include "yabloc_image_processing/segment_filter/segment_filter.hpp"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::segment_filter
{
SegmentFilter::SegmentFilter()
: Node("segment_filter"),
  synchro_subscriber_(this, "~/input/line_segments_cloud", "~/input/graph_segmented"),
  tf_subscriber_(this->get_clock())
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  auto cb = std::bind(&SegmentFilter::execute, this, _1, _2);
  synchro_subscriber_.set_callback(std::move(cb));

  pub_classified_cloud_ = create_publisher<PointCloud2>("~/output/classified_line_segments", 10);
}

void SegmentFilter::execute(const PointCloud2 & line_segments_msg, const Image & segment_msg)
{
  const rclcpp::Time stamp = line_segments_msg.header.stamp;

  pcl::PointCloud<pcl::PointNormal>::Ptr line_segments_cloud{
    new pcl::PointCloud<pcl::PointNormal>()};
  cv::Mat mask_image = common::decompress_to_cv_mat(segment_msg);
  pcl::fromROSMsg(line_segments_msg, *line_segments_cloud);

  const std::set<int> indices = filter_by_mask(mask_image, *line_segments_cloud);

  pcl::PointCloud<pcl::PointXYZLNormal> combined_edges;
  for (int index = 0; index < line_segments_cloud->size(); ++index) {
    const cl::PointNormal & pn = line_segments_cloud->at(index);
    pcl::PointXYZLNormal pln;
    pln.getVector3fMap() = pn.getVector3fMap();
    pln.getNormalVector3fMap() = pn.getNormalVector3fMap();
    if (indices.count(index) > 0) {
      pln.label = 255;
    } else {
      pln.label = 0;
    }
    combined_edges.push_back(pln);
  }
  common::publish_cloud(*pub_projected_cloud_, combined_edges, stamp);
}

std::set<ushort> get_unique_pixel_value(cv::Mat & image)
{
  // `image` is a set of ushort.
  // The purpose is to find the unique set of values contained in `image`.
  // For example, if `image` is {0,1,2,0,1,2,3}, this function returns {0,1,2,3}.

  if (image.depth() != CV_16U) throw std::runtime_error("image's depth must be ushort");

  auto begin = image.begin<ushort>();
  auto last = std::unique(begin, image.end<ushort>());
  std::sort(begin, last);
  last = std::unique(begin, last);
  return std::set<ushort>(begin, last);
}

std::set<int> SegmentFilter::filter_by_mask(
  const cv::Mat & mask, const pcl::PointCloud<pcl::PointNormal> & edges)
{
  // Create line image and assign different color to each segment.
  cv::Mat line_image = cv::Mat::zeros(mask.size(), CV_16UC1);
  for (size_t i = 0; i < edges.size(); i++) {
    auto & pn = edges.at(i);
    Eigen::Vector3f p1 = pn.getVector3fMap();
    Eigen::Vector3f p2 = pn.getNormalVector3fMap();
    cv::Scalar color = cv::Scalar::all(i + 1);
    cv::line(
      line_image, cv::Point2i(p1.x(), p1.y()), cv::Point2i(p2.x(), p2.y()), color, 1,
      cv::LineTypes::LINE_4);
  }

  cv::Mat mask_image;
  mask.convertTo(mask_image, CV_16UC1);
  cv::threshold(mask_image, mask_image, 1, std::numeric_limits<ushort>::max(), cv::THRESH_BINARY);

  // TODO(KYabuuchi) Using boost::geometry is more intuitive.
  // https://boostjp.github.io/tips/geometry.html#disjoint

  // And operator
  cv::Mat masked_line;
  cv::bitwise_and(mask_image, line_image, masked_line);
  std::set<ushort> pixel_values = get_unique_pixel_value(masked_line);

  // Extract edges within masks
  std::set<int> reliable_indices;
  for (size_t i = 0; i < edges.size(); i++) {
    if (pixel_values.count(i + 1) != 0) reliable_indices.insert(i);
  }

  return reliable_indices;
}

}  // namespace yabloc::segment_filter
