#ifndef NDT_SCAN_MATCHER__MAP_LOADER_HPP_
#define NDT_SCAN_MATCHER__MAP_LOADER_HPP_

// Copyright 2022 The Autoware Contributors
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

#include "utils.hpp"

#include <ndt_scan_matcher/metadata.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <future>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

namespace loc
{

template <typename PointT>
class MapLoader
{
  typedef pcl::PointCloud<PointT> PclCloudType;
  typedef typename PclCloudType::Ptr PclCloudPtr;

public:
  MapLoader()
  {
    thread_num_ = 1;

    setThreadNum(4);
  }

  explicit MapLoader(std::shared_ptr<PCDMetadata> metadata);

  void import(const std::string & metadata_path, const std::string & pcd_dir)
  {
    metadata_.reset(new PCDMetadata);
    metadata_->import(metadata_path, pcd_dir);
  }

  // Retrieve a map whose key is segment indices and value is a pointer to
  // the point cloud loaded from the segment PCD
  void load(
    float pos_x, float pos_y, float radius, const std::vector<std::string> & cached_ids,
    std::map<std::string, PclCloudPtr> & pcd_to_add, std::set<std::string> & ids_to_remove);

  // Find the names of PCDs to be added/removed but not load them from files
  void query(
    float pos_x, float pos_y, float radius, const std::vector<std::string> & cached_ids,
    std::map<std::string, std::string> & pcd_to_add, std::set<std::string> & ids_to_remove);

  // Warper around loadPCDFile
  int load(const std::string & path, PclCloudType & cloud);

  void setThreadNum(int thread_num)
  {
    sync();

    if (thread_num > 1) {
      // One more thread for the load manager
      thread_num += 1;
      thread_num_ = thread_num;
      thread_futs_.resize(thread_num_);
    }
  }

  // Wait for all running threads to finish
  inline void sync()
  {
    if (load_manager_fut_.valid()) {
      load_manager_fut_.wait();
    }

    for (auto & tf : thread_futs_) {
      if (tf.valid()) {
        tf.wait();
      }
    }

    last_check_tid_ = -1;
  }

  // Parallel load but pcd files come in stream
  void parallel_load_setup(
    float x, float y, float radius, const std::vector<std::string> & cached_ids);

  // Return false if no more pcd is loaded, true otherwise
  bool get_next_loaded_pcd(std::string & map_id, PclCloudPtr & loaded_pcd);

  std::set<std::string> get_pcd_id_to_remove() { return pcd_to_remove_; }

  // Clear the current stream
  void parallel_load_clear();

private:
  bool loadThread(const std::string & map_id, const std::string & path);
  bool loadManagerThread();

  // Return the index of an idle thread, which is not running any
  // job, or has already finished its job and waiting for a join.
  inline int get_idle_tid()
  {
    int tid = (last_check_tid_ == thread_num_ - 1) ? 0 : last_check_tid_ + 1;
    std::chrono::microseconds span(50);

    // Loop until an idle thread is found
    while (true) {
      // Return immediately if a thread that has not been given a job is found
      if (!thread_futs_[tid].valid()) {
        last_check_tid_ = tid;
        return tid;
      }

      // If no such thread is found, wait for the current thread to finish its job
      if (thread_futs_[tid].wait_for(span) == std::future_status::ready) {
        last_check_tid_ = tid;
        return tid;
      }

      // If the current thread has not finished its job, check the next thread
      tid = (tid == thread_num_ - 1) ? 0 : tid + 1;
    }
  }

  std::shared_ptr<PCDMetadata> metadata_;

  int thread_num_;
  int last_check_tid_;
  std::vector<std::future<bool>> thread_futs_;
  std::future<bool> load_manager_fut_;
  std::queue<std::pair<std::string, PclCloudPtr>> output_queue_;
  std::map<std::string, std::string> pcd_to_add_;
  std::set<std::string> pcd_to_remove_;
  std::mutex queue_mtx_;
  size_t loaded_counter_;
};

}  // namespace loc

#endif  // NDT_SCAN_MATCHER__MAP_LOADER_HPP_
