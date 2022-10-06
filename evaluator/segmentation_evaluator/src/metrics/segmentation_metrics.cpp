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

#include "segmentation_evaluator/metrics/segmentation_metrics.hpp"

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "boost/container_hash/hash.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace segmentation_diagnostics
{
namespace metrics
{

class HashFun
{
public:
  size_t operator()(const std::vector<float> & p) const { return boost::hash_value(p); }
};

Stat<double> updatePclStats(
  const PointCloud2 & pcl_to_eval, const PointCloud2 & pcl_gt_negative_cls,
  const PointCloud2 & pcl_gt_positive_cls, const Stat<double> stat_prev)
{
  Stat<double> stat(stat_prev);
  std::vector<int> conf_mat;
  conf_mat = computeConfusionMatrix(pcl_to_eval, pcl_gt_negative_cls, pcl_gt_positive_cls);
  stat.setConfustionMatPerCloud(conf_mat);
  return stat;
}

float r(float v) { return floor(v * 1000.0) / 1000.0; }

std::vector<int> computeConfusionMatrix(
  const PointCloud2 & pcl_to_eval, const PointCloud2 & pcl_gt_negative_cls,
  const PointCloud2 & pcl_gt_positive_cls)
{
  int tp = 0, fn = 0, fp = 0, tn = 0;
  const size_t number_of_points = pcl_to_eval.width;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_pred(pcl_to_eval, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_pred(pcl_to_eval, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_pred(pcl_to_eval, "z");

  const size_t number_of_points_ground = pcl_gt_negative_cls.width;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_g(pcl_gt_negative_cls, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_g(pcl_gt_negative_cls, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_g(pcl_gt_negative_cls, "z");

  const size_t number_of_points_obj = pcl_gt_positive_cls.width;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_o(pcl_gt_positive_cls, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_o(pcl_gt_positive_cls, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_o(pcl_gt_positive_cls, "z");

  std::unordered_set<std::vector<float>, boost::hash<std::vector<float> > > set_pcl_gt_negative_cls,
    set_pcl_gt_positive_cls;
  for (size_t i = 0; i < number_of_points_ground; ++i, ++iter_x_g, ++iter_y_g, ++iter_z_g) {
    std::vector<float> point({r(*iter_x_g), r(*iter_y_g), r(*iter_z_g)});
    set_pcl_gt_negative_cls.insert(point);
  }
  for (size_t i = 0; i < number_of_points_obj; ++i, ++iter_x_o, ++iter_y_o, ++iter_z_o) {
    std::vector<float> point({r(*iter_x_o), r(*iter_y_o), r(*iter_z_o)});
    set_pcl_gt_positive_cls.insert(point);
  }

  for (size_t i = 0; i < number_of_points; ++i, ++iter_x_pred, ++iter_y_pred, ++iter_z_pred) {
    std::vector<float> point({r(*iter_x_pred), r(*iter_y_pred), r(*iter_z_pred)});

    int is_ground = set_pcl_gt_negative_cls.count(point);
    int is_obj = set_pcl_gt_positive_cls.count(point);
    if (is_ground) {
      fp += 1;
    } else if (is_obj) {
      tp += 1;
    }
  }
  fn = number_of_points_obj - tp;
  tn = number_of_points_ground - fp;

  return {tp, fn, fp, tn};
}

}  // namespace metrics
}  // namespace segmentation_diagnostics
