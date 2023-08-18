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

#include "yabloc_image_processing/line_segments_projector/line_segments_projector.hpp"

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::line_segments_projector
{
LineSegmentsProjector::LineSegmentsProjector()
: Node("line_segments_projector"),
  image_size_(declare_parameter<int>("image_size")),
  max_range_(declare_parameter<float>("max_range")),
  min_segment_length_(declare_parameter<float>("min_segment_length")),
  max_segment_distance_(declare_parameter<float>("max_segment_distance")),
  max_lateral_distance_(declare_parameter<float>("max_lateral_distance")),
  info_(this),
  tf_subscriber_(this->get_clock())
{
  sub_classified_line_segments_ = create_subscription<PointCloud2>(
    "~/input/classified_line_segments", 10,
    [this](PointCloud2::ConstSharedPtr msg) { this->on_classified_line_segments(msg); });

  pub_projected_cloud_ =
    create_publisher<PointCloud2>("~/output/projected_line_segments_cloud", 10);
  pub_image_ = create_publisher<Image>("~/debug/projected_image", 10);
}

cv::Point2i LineSegmentsProjector::to_cv_point(const Eigen::Vector3f & v) const
{
  cv::Point pt;
  pt.x = -v.y() / max_range_ * image_size_ * 0.5f + image_size_ / 2;
  pt.y = -v.x() / max_range_ * image_size_ * 0.5f + image_size_;
  return pt;
}

bool LineSegmentsProjector::define_project_func()
{
  if (project_func_) return true;

  if (info_.is_camera_info_nullopt()) return false;
  Eigen::Matrix3f intrinsic_inv = info_.intrinsic().inverse();

  std::optional<Eigen::Affine3f> camera_extrinsic =
    tf_subscriber_(info_.get_frame_id(), "base_link");
  if (!camera_extrinsic.has_value()) return false;

  const Eigen::Vector3f t = camera_extrinsic->translation();
  const Eigen::Quaternionf q(camera_extrinsic->rotation());

  // TODO(KYabuuchi) This will take into account ground tilt and camera vibration someday.
  project_func_ = [intrinsic_inv, q,
                   t](const Eigen::Vector3f & u) -> std::optional<Eigen::Vector3f> {
    Eigen::Vector3f u3(u.x(), u.y(), 1);
    Eigen::Vector3f u_bearing = (q * intrinsic_inv * u3).normalized();
    if (u_bearing.z() > -0.01) return std::nullopt;
    float u_distance = -t.z() / u_bearing.z();
    Eigen::Vector3f v;
    v.x() = t.x() + u_bearing.x() * u_distance;
    v.y() = t.y() + u_bearing.y() * u_distance;
    v.z() = 0;
    return v;
  };
  return true;
}

void LineSegmentsProjector::on_classified_line_segments(
  const PointCloud2::ConstSharedPtr & line_segments_msg)
{
  if (!define_project_func()) {
    using namespace std::literals::chrono_literals;
    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(), *get_clock(), (1000ms).count(), "project_func cannot be defined");
    return;
  }

  const rclcpp::Time stamp = line_segments_msg->header.stamp;

  pcl::PointCloud<pcl::PointXYZLNormal>::Ptr line_segments_cloud{
    new pcl::PointCloud<pcl::PointXYZLNormal>()};
  pcl::fromROSMsg(*line_segments_msg, *line_segments_cloud);

  pcl::PointCloud<pcl::PointXYZLNormal> valid_line_segments;
  pcl::PointCloud<pcl::PointXYZLNormal> invalid_line_segments;

  for (size_t index = 0; index < line_segments_cloud->size(); ++index) {
    const pcl::PointXYZLNormal & pln = line_segments_cloud->at(index);
    if (pln.label == 255) {
      valid_line_segments.push_back(pln);
    } else {
      invalid_line_segments.push_back(pln);
    }
  }

  pcl::PointCloud<pcl::PointXYZLNormal> valid_edges = project_lines(valid_line_segments);
  pcl::PointCloud<pcl::PointXYZLNormal> invalid_edges = project_lines(invalid_line_segments);

  // Projected line segments
  {
    pcl::PointCloud<pcl::PointXYZLNormal> combined_edges;
    for (const auto & pln : valid_edges) {
      combined_edges.push_back(pln);
    }
    for (const auto & pln : invalid_edges) {
      combined_edges.push_back(pln);
    }
    common::publish_cloud(*pub_projected_cloud_, combined_edges, stamp);
  }

  // Image
  {
    cv::Mat projected_image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
    for (auto & pn : valid_edges) {
      cv::Point2i p1 = to_cv_point(pn.getVector3fMap());
      cv::Point2i p2 = to_cv_point(pn.getNormalVector3fMap());
      cv::line(projected_image, p1, p2, cv::Scalar(100, 100, 255), 4, cv::LineTypes::LINE_8);
    }
    for (auto & pn : invalid_edges) {
      cv::Point2i p1 = to_cv_point(pn.getVector3fMap());
      cv::Point2i p2 = to_cv_point(pn.getNormalVector3fMap());
      cv::line(projected_image, p1, p2, cv::Scalar(200, 200, 200), 3, cv::LineTypes::LINE_8);
    }
    common::publish_image(*pub_image_, projected_image, stamp);
  }
}

bool LineSegmentsProjector::is_near_element(
  const pcl::PointXYZLNormal & pn, pcl::PointXYZLNormal & truncated_pn) const
{
  float min_distance = std::min(pn.x, pn.normal_x);
  float max_distance = std::max(pn.x, pn.normal_x);
  if (min_distance > max_segment_distance_) return false;
  if (max_distance < max_segment_distance_) {
    truncated_pn = pn;
    return true;
  }

  truncated_pn = pn;
  Eigen::Vector3f t = pn.getVector3fMap() - pn.getNormalVector3fMap();
  float not_zero_tx = t.x() > 0 ? std::max(t.x(), 1e-3f) : std::min(t.x(), -1e-3f);
  float lambda = (max_segment_distance_ - pn.x) / not_zero_tx;
  Eigen::Vector3f m = pn.getVector3fMap() + lambda * t;
  if (pn.x > pn.normal_x)
    truncated_pn.getVector3fMap() = m;
  else
    truncated_pn.getNormalVector3fMap() = m;
  return true;
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

pcl::PointCloud<pcl::PointXYZLNormal> LineSegmentsProjector::project_lines(
  const pcl::PointCloud<pcl::PointXYZLNormal> & points) const
{
  pcl::PointCloud<pcl::PointXYZLNormal> projected_points;
  for (const auto & pn : points) {
    std::optional<Eigen::Vector3f> opt1 = project_func_(pn.getVector3fMap());
    std::optional<Eigen::Vector3f> opt2 = project_func_(pn.getNormalVector3fMap());
    if (!opt1.has_value()) continue;
    if (!opt2.has_value()) continue;

    // If line segment has shorter length than config, it is excluded
    if (min_segment_length_ > 0) {
      float length = (opt1.value() - opt2.value()).norm();
      if (length < min_segment_length_) continue;
    }
    if (max_lateral_distance_ > 0) {
      float abs_lateral1 = std::abs(opt1.value().y());
      float abs_lateral2 = std::abs(opt2.value().y());
      if (std::min(abs_lateral1, abs_lateral2) > max_lateral_distance_) continue;
    }

    pcl::PointXYZLNormal xyz;
    xyz.x = opt1->x();
    xyz.y = opt1->y();
    xyz.z = opt1->z();
    xyz.normal_x = opt2->x();
    xyz.normal_y = opt2->y();
    xyz.normal_z = opt2->z();
    xyz.label = pn.label;

    //
    pcl::PointXYZLNormal truncated_xyz = xyz;
    if (max_segment_distance_ > 0)
      if (!is_near_element(xyz, truncated_xyz)) continue;

    projected_points.push_back(truncated_xyz);
  }
  return projected_points;
}

}  // namespace yabloc::line_segments_projector
