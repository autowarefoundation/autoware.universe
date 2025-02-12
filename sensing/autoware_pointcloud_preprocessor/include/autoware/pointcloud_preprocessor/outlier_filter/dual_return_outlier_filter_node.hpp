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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

class DualReturnOutlierFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr visibility_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr noise_cloud_pub_;

private:
  void onVisibilityChecker(DiagnosticStatusWrapper & stat);
  Updater updater_{this};
  double visibility_ = -1.0f;
  double weak_first_distance_ratio_;
  double general_distance_ratio_;
  int weak_first_local_noise_threshold_;
  double visibility_error_threshold_;
  double visibility_warn_threshold_;
  int vertical_bins_;
  float max_azimuth_diff_;
  std::string roi_mode_;
  float x_max_;
  float x_min_;
  float y_max_;
  float y_min_;
  float z_max_;
  float z_min_;

  float min_azimuth_deg_;
  float max_azimuth_deg_;
  float max_distance_;

  std::unordered_map<std::string, uint8_t> roi_mode_map_ = {
    {"No_ROI", 0},
    {"Fixed_xyz_ROI", 1},
    {"Fixed_azimuth_ROI", 2},
  };

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit DualReturnOutlierFilterComponent(const rclcpp::NodeOptions & options);
};

}  // namespace autoware::pointcloud_preprocessor

// clang-format off
#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__OUTLIER_FILTER__DUAL_RETURN_OUTLIER_FILTER_NODE_HPP_  // NOLINT
// clang-format on
