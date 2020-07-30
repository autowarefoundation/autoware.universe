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

#include "pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

namespace pointcloud_preprocessor
{
bool RingOutlierFilterNodelet::child_init(ros::NodeHandle & nh, bool & has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<
    dynamic_reconfigure::Server<pointcloud_preprocessor::RingOutlierFilterConfig>>(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::RingOutlierFilterConfig>::CallbackType f =
    boost::bind(&RingOutlierFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void RingOutlierFilterNodelet::filter(
  const PointCloud2::ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PointCloud<pcl::PointXYZIRADT>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZIRADT>);
  pcl::fromROSMsg(*input, *pcl_input);

  if (pcl_input->points.empty()) {
    return;
  }
  std::vector<pcl::PointCloud<pcl::PointXYZIRADT>> pcl_input_ring_array;
  pcl_input_ring_array.resize(128);  // TODO
  for (const auto& p : pcl_input->points) {
    pcl_input_ring_array.at(p.ring).push_back(p);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_output->points.reserve(pcl_input->points.size());

  pcl::PointCloud<pcl::PointXYZ> pcl_tmp;
  pcl::PointXYZ p;
  for (const auto & ring_pointcloud : pcl_input_ring_array) {
    if (ring_pointcloud.points.size() < 2) {
      continue;
    }

    for (auto iter = std::begin(ring_pointcloud.points);
         iter != std::end(ring_pointcloud.points) - 1; ++iter) {
      p.x = iter->x;
      p.y = iter->y;
      p.z = iter->z;
      pcl_tmp.points.push_back(p);
      // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08) {
      const double min_dist = std::min(iter->distance, (iter + 1)->distance);
      const double max_dist = std::max(iter->distance, (iter + 1)->distance);
      if (min_dist > 0 && max_dist > 0 && max_dist / min_dist < distance_ratio_) {
        continue;
      } else {
        if (
          pcl_tmp.points.size() > num_points_threshold_ &&
          std::sqrt(
            std::pow(pcl_tmp.points.front().x - pcl_tmp.points.back().x, 2.0) +
            std::pow(pcl_tmp.points.front().y - pcl_tmp.points.back().y, 2.0) +
            std::pow(pcl_tmp.points.front().z - pcl_tmp.points.back().z, 2.0)) >=
            object_length_threshold_) {
          for (const auto & tmp_p : pcl_tmp.points) {
            pcl_output->points.push_back(tmp_p);
          }
        }
        pcl_tmp.points.clear();
      }
    }
  }

  // TODO delete test code
  // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointXYZ p;
  // for (const auto& ring_pointcloud : pcl_input_ring_array) {
  //   if(ring_pointcloud.points.size() < 10) {
  //     continue;
  //   }
  //   bool is_next_ok = false;
  //
  //   for (auto iter = std::begin(ring_pointcloud.points); iter != std::end(ring_pointcloud.points)-1; ++iter) {
  //     if(std::abs(iter->azimuth - (iter+1)->azimuth) <= 50  && iter->distance >= 0.5 && (iter+1)->distance >= 0.5 &&
  //     std::abs(iter->distance - (iter+1)->distance) <= 0.5) {
  //         p.x = iter->x;
  //         p.y = iter->y;
  //         p.z = iter->z;
  //         pcl_tmp->points.push_back(p);
  //         is_next_ok = true;
  //     }
  //     else if(is_next_ok) {
  //       p.x = iter->x;
  //       p.y = iter->y;
  //       p.z = iter->z;
  //       pcl_tmp->points.push_back(p);
  //
  //       if(pcl_tmp->points.size() > 3 && std::sqrt(std::pow(pcl_tmp->points.front().x -
  //       pcl_tmp->points.back().x, 2.0) + std::pow(pcl_tmp->points.front().y - pcl_tmp->points.back().y, 2.0)) >= 0.1)
  //       {
  //         pcl_output->points.insert(std::end(pcl_output->points), std::begin(pcl_tmp->points),
  //         std::end(pcl_tmp->points));
  //       }
  //       pcl_tmp->points.clear();
  //       is_next_ok = false;
  //     }
  //     else {
  //       is_next_ok = false;
  //     }
  //   }
  // }

  // pcl::PointCloud<pcl::PointXYZ> pcl_tmp;
  // std::vector<pcl::PointCloud<pcl::PointXYZ>> pcl_tmp_array;
  // std::vector<pcl::PointXYZ> centroid_array;
  // std::vector<int> index_array;
  // pcl::PointXYZ p;
  // for (const auto& ring_pointcloud : pcl_input_ring_array) {
  //   if(ring_pointcloud.points.size() < 2) {
  //     continue;
  //   }
  //
  //   for (auto iter = std::begin(ring_pointcloud.points); iter != std::end(ring_pointcloud.points)-1; ++iter) {
  //     p.x = iter->x;
  //     p.y = iter->y;
  //     p.z = iter->z;
  //     // if(std::abs(iter->distance - (iter+1)->distance) <= std::sqrt(iter->distance) * 0.08) {
  //     const double min_dist = std::min(iter->distance, (iter+1)->distance);
  //     const double max_dist = std::max(iter->distance, (iter+1)->distance);
  //     if(min_dist > 0 && max_dist > 0 && min_dist/max_dist < 0.97 ) {
  //       pcl_tmp.points.push_back(p);
  //       continue;
  //     }
  //     else {
  //       pcl_tmp.points.push_back(p);
  //
  //       if(!pcl_tmp.points.empty()) {
  //
  //         pcl_tmp_array.push_back(pcl_tmp);
  //         index_array.push_back(-1);
  //
  //         pcl::PointXYZ centroid;
  //         for (const auto& tmp_p : pcl_tmp.points) {
  //           centroid.x += tmp_p.x;
  //           centroid.y += tmp_p.y;
  //           centroid.z += tmp_p.z;
  //         }
  //         centroid.x = centroid.x / pcl_tmp.points.size();
  //         centroid.y = centroid.y / pcl_tmp.points.size();
  //         centroid.z = centroid.z / pcl_tmp.points.size();
  //         centroid_array.push_back(centroid);
  //         pcl_tmp.points.clear();
  //       }
  //
  //     }
  //   }
  // }

  // if(pcl_tmp_array.size() < 2) {
  //   return;
  // }

  // std::vector<pcl::PointCloud<pcl::PointXYZ>> pcl_array;
  // pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
  // for (size_t i = 0; i < pcl_tmp_array.size()-1; ++i) {
  //   int index = index_array.at(i);
  //   if(index == -1) {
  //     index = pcl_array.size();
  //     index_array.at(i) = index;
  //     pcl_array.push_back(pcl_tmp_array.at(i));
  //   }
  //
  //   for (size_t j = i+1; j < pcl_tmp_array.size(); ++j) {
  //     double dist = std::sqrt(std::pow(centroid_array.at(i).x - centroid_array.at(j).x, 2.0) +
  //     std::pow(centroid_array.at(i).y - centroid_array.at(j).y, 2.0) + std::pow(centroid_array.at(i).z -
  //     centroid_array.at(j).z, 2.0)); if(dist <= 0.5) {
  //       index_array.at(j) = index;
  //       pcl_array.at(index).points.insert(std::end(pcl_array.at(index).points),
  //       std::begin(pcl_tmp_array.at(j).points), std::end(pcl_tmp_array.at(j).points));
  //     }
  //   }
  // }
  //
  // for (const auto& cloud : pcl_array) {
  //   if(cloud.points.size() > 3) {
  //     pcl_output->points.insert(std::end(pcl_output->points), std::begin(cloud.points), std::end(cloud.points));
  //   }
  // }

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void RingOutlierFilterNodelet::subscribe() { Filter::subscribe(); }

void RingOutlierFilterNodelet::unsubscribe() { Filter::unsubscribe(); }

void RingOutlierFilterNodelet::config_callback(
  pointcloud_preprocessor::RingOutlierFilterConfig & config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (distance_ratio_ != config.distance_ratio) {
    distance_ratio_ = config.distance_ratio;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new distance ratio to: %f.", getName().c_str(),
      config.distance_ratio);
  }
  if (object_length_threshold_ != config.object_length_threshold) {
    object_length_threshold_ = config.object_length_threshold;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new object length threshold to: %f.", getName().c_str(),
      config.object_length_threshold);
  }
  if (num_points_threshold_ != config.num_points_threshold) {
    num_points_threshold_ = config.num_points_threshold;
    NODELET_DEBUG(
      "[%s::config_callback] Setting new num_points_threshold to: %d.", getName().c_str(),
      config.num_points_threshold);
  }
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit
  // from Filter
  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG("[config_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG(
      "[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::RingOutlierFilterNodelet, nodelet::Nodelet);
