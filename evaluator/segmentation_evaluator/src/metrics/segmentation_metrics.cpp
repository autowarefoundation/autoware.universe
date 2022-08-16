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

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "boost/container_hash/hash.hpp"

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
  const PointCloud2 & pcl_to_eval, const PointCloud2 & pcl_gt_ground,
  const PointCloud2 & pcl_gt_obj, const Stat<double> stat_prev, PointCloud2 & pcl_no_ex)
{
  Stat<double> stat(stat_prev);
  std::vector<int> conf_mat;
  conf_mat = computeConfusionMatrix(pcl_to_eval, pcl_gt_ground, pcl_gt_obj, pcl_no_ex);
  stat.setConfustionMatPerCloud(conf_mat);
  stat.printConfMat();
  stat.printConfMatAccu();
  return stat;
}

float r(float v)
{
  return floor(v*1000.0) / 1000.0;
}

std::vector<int> computeConfusionMatrix(
  const PointCloud2 & pcl_to_eval, const PointCloud2 & pcl_gt_ground,
  const PointCloud2 & pcl_gt_obj, PointCloud2 & pcl_no_ex)
{
  // std::cout << pcl_gt_ground.row_step << " + " << pcl_gt_obj.row_step << " = "
  //           << pcl_gt_ground.row_step + pcl_gt_obj.row_step << "    !eval: " << pcl_to_eval.row_step
  //           << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_no_ex_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_no_ex_xyz->points.reserve(pcl_to_eval.row_step);

  int tp = 0, fn = 0, fp = 0, tn = 0;
  const size_t number_of_points = pcl_to_eval.width;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_pred(pcl_to_eval, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_pred(pcl_to_eval, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_pred(pcl_to_eval, "z");

  const size_t number_of_points_ground = pcl_gt_ground.width;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_g(pcl_gt_ground, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_g(pcl_gt_ground, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_g(pcl_gt_ground, "z");

  const size_t number_of_points_obj = pcl_gt_obj.width;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x_o(pcl_gt_obj, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y_o(pcl_gt_obj, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z_o(pcl_gt_obj, "z");

  std::unordered_set<std::vector<float>, boost::hash<std::vector<float> > > set_pcl_gt_ground,
    set_pcl_gt_obj;
  for (size_t i = 0; i < number_of_points_ground; ++i, ++iter_x_g, ++iter_y_g, ++iter_z_g) {
    std::vector<float> point({r(*iter_x_g), r(*iter_y_g), r(*iter_z_g)});
    set_pcl_gt_ground.insert(point);
  }
  for (size_t i = 0; i < number_of_points_obj; ++i, ++iter_x_o, ++iter_y_o, ++iter_z_o) {
    std::vector<float> point({r(*iter_x_o), r(*iter_y_o), r(*iter_z_o)});
    set_pcl_gt_obj.insert(point);
  }
  // std::cout << set_pcl_gt_ground.size() << " + " << set_pcl_gt_obj.size() << " = "
  //           << set_pcl_gt_ground.size() + set_pcl_gt_obj.size()
  //           << "    eval: " << pcl_to_eval.width << std::endl;

  int cnt_zero = 0;
  int cnt_non_ex = 0;
  for (size_t i = 0; i < number_of_points; ++i, ++iter_x_pred, ++iter_y_pred, ++iter_z_pred) {
    std::vector<float> point({r(*iter_x_pred), r(*iter_y_pred), r(*iter_z_pred)});

    int is_ground = set_pcl_gt_ground.count(point);
    int is_obj = set_pcl_gt_obj.count(point);
    if (point[0] == 0 && point[1] == 0 && point[2] == 0) {
      cnt_zero++;
    } else if (is_ground == 0 && is_obj == 0) {
      cnt_non_ex++;
      pcl_no_ex_xyz->push_back({point[0], point[1], point[2]});
    } else if (is_ground) {
      fp += 1;
    } else if (is_obj) {
      tp += 1;
    } else {
      std::cout << "No point";
    }
  }
  fn = number_of_points_obj - tp;
  tn = number_of_points_ground - fp;

  // std::cout << "ZEROS in pred: " << cnt_zero << "\n";
  // std::cout << "NONEX in pred: " << cnt_non_ex << "\n";
  std::cout << "Accuracy: " << float(tp+tn)/float(tp+tn+fn+fp) << "\n";
  std::cout << "Precision: " << float(tp)/float(tp+fp) << "\n";  
  std::cout << "Recall: " << float(tp)/float(tp+fn) << "\n";  

  pcl::toROSMsg(*pcl_no_ex_xyz, pcl_no_ex);
  pcl_no_ex.set__header(pcl_to_eval.header);

  // for (size_t i = 0; i < number_of_points_ground; ++i, ++iter_x_pred, ++iter_y_pred,
  // ++iter_z_pred) {
  //   float x = *iter_x_pred;
  //   float y = *iter_y_pred;
  //   float z = *iter_z_pred;
  //   std::vector<float> point({x, y, z});

  //   if (map_pcl[point] == 127 || map_pcl[point] == 10) {
  //     fp += 1;
  //   } else {
  //     tp += 1;
  //   }
  //   map_pcl.erase(point);
  // }

  // for (auto & point : map_pcl) {
  //   if (map_pcl[point.first] == 127 || map_pcl[point.first] == 10) {
  //     tn += 1;
  //   } else {
  //     fn += 1;
  //   }
  // }

  // std::cout << tp << "\n";

  return {tp, fn, fp, tn};
}

}  // namespace metrics
}  // namespace segmentation_diagnostics
