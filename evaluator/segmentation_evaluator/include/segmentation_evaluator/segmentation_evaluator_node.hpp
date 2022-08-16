// Copyright 2021 Tier IV, Inc.
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

#ifndef SEGMENTATION_EVALUATOR__SEGMENTATION_EVALUATOR_NODE_HPP_
#define SEGMENTATION_EVALUATOR__SEGMENTATION_EVALUATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "segmentation_evaluator/metrics_calculator.hpp"
#include "segmentation_evaluator/stat.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace segmentation_diagnostics
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

/**
 * @brief Node for segmentation evaluation
 */
class SegmentationEvaluatorNode : public rclcpp::Node
{
public:
  explicit SegmentationEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~SegmentationEvaluatorNode();

  /**
   * @brief synchronized callback on two point clouds
   * @param [in] msg Poind cloud message
   * @param [in] msg_gt_ground Ground truth point cloud with ground
   * @param [in] msg_gt_obj Ground truth point cloud with objects
   */
  void syncCallback(
    const PointCloud2::ConstSharedPtr & msg, const PointCloud2::ConstSharedPtr & msg_gt_ground,
    const PointCloud2::ConstSharedPtr & msg_gt_obj);

  /**
   * @brief publish the given metric statistic
   */
  DiagnosticStatus generateDiagnosticStatus(
    const Metric & metric, const Stat<double> & metric_stat) const;

private:
  geometry_msgs::msg::Pose getCurrentEgoPose() const;

  // ROS
  message_filters::Subscriber<PointCloud2> pcl_sub_, pcl_gt_ground_sub_, pcl_gt_obj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_no_ex_pub_;

  typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> SyncExact;
  SyncExact sync_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // Parameters
  std::string output_file_str_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  std::deque<rclcpp::Time> stamps_;
  std::array<std::deque<Stat<double>>, static_cast<size_t>(Metric::SIZE)> metric_stats_;
  std::unordered_map<Metric, Stat<double>> metrics_dict_;
};
}  // namespace segmentation_diagnostics

#endif  // SEGMENTATION_EVALUATOR__SEGMENTATION_EVALUATOR_NODE_HPP_
