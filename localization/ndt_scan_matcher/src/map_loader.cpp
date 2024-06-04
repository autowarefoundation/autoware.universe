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

#include <ndt_scan_matcher/map_loader.hpp>

namespace loc
{

template <typename PointT>
MapLoader<PointT>::MapLoader(std::shared_ptr<PCDMetadata> metadata) : metadata_(metadata)
{
}

template <typename PointT>
void MapLoader<PointT>::load(
  float pos_x, float pos_y, float radius, const std::vector<std::string> & cached_ids,
  std::map<std::string, PclCloudPtr> & pcd_to_load, std::set<std::string> & pcd_to_remove)
{
  pcd_to_load.clear();
  pcd_to_remove.clear();

  std::map<std::string, std::string> pcd_to_load_and_path;

  query(pos_x, pos_y, radius, cached_ids, pcd_to_load_and_path, pcd_to_remove);

  // Parallel load
  for (auto & it : pcd_to_load_and_path) {
    auto & target_cloud = pcd_to_load[it.first];

    target_cloud.reset(new PclCloudType);

    if (pcl::io::loadPCDFile(it.second, *target_cloud)) {
      std::cerr << "Error: Failed to load file " << it.second << std::endl;
    }
  }
}

template <typename PointT>
bool MapLoader<PointT>::loadThread(const std::string & map_id, const std::string & path)
{
  PclCloudPtr new_cloud(new PclCloudType);

  if (pcl::io::loadPCDFile(path, *new_cloud)) {
    return false;
  }

  // Push the new cloud to the output queue
  queue_mtx_.lock();
  output_queue_.push(std::make_pair(map_id, new_cloud));
  queue_mtx_.unlock();

  return true;
}

template <typename PointT>
void MapLoader<PointT>::query(
  float pos_x, float pos_y, float radius, const std::vector<std::string> & cached_ids,
  std::map<std::string, std::string> & pcd_to_add, std::set<std::string> & ids_to_remove)
{
  // Create a set of should remove indices, which is originally a copy of @cached_ids
  ids_to_remove.clear();
  ids_to_remove.insert(cached_ids.begin(), cached_ids.end());

  // Get the indices of segments within the specified area
  std::list<SegmentIndex> contained_map_id;

  queryContainedSegmentIdx(pos_x, pos_y, radius, *metadata_, contained_map_id);

  // Look for the contained segments from the metadata of pcd
  for (auto & seg_id : contained_map_id) {
    // If the segment is already in the cached ids, remove it from the should_remove
    auto map_id = seg_id.to_string();
    auto it = ids_to_remove.find(map_id);

    if (it != ids_to_remove.end()) {
      ids_to_remove.erase(it);
      continue;
    }

    // Otherwise, look for the segment from the list of segment pcd
    auto pcd_it = metadata_->find(seg_id);

    // Skip if the segment pcd does not exist
    if (pcd_it != metadata_->end()) {
      pcd_to_add[map_id] = pcd_it->second;
    }
  }
}

template <typename PointT>
int MapLoader<PointT>::load(const std::string & path, PclCloudType & cloud)
{
  return pcl::io::loadPCDFile(path, cloud);
}

template <typename PointT>
void MapLoader<PointT>::parallel_load_setup(
  float x, float y, float radius, const std::vector<std::string> & cached_ids)
{
  // Clear any remaining garbage from the previous run
  parallel_load_clear();

  // Query the indices of PCDs to be loaded/removed
  query(x, y, radius, cached_ids, pcd_to_add_, pcd_to_remove_);

  // Start loading right now
  load_manager_fut_ = std::async(std::launch::async, &MapLoader<PointT>::loadManagerThread, this);
}

template <typename PointT>
bool MapLoader<PointT>::get_next_loaded_pcd(std::string & map_id, PclCloudPtr & loaded_pcd)
{
  // Return false if no remaining pcd to load
  if (pcd_to_add_.size() == loaded_counter_) {
    return false;
  }

  // Wait until some pcds are loaded
  while (output_queue_.empty()) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  auto & head = output_queue_.front();

  map_id = head.first;
  loaded_pcd = head.second;

  // Remove the head of the queue
  queue_mtx_.lock();
  output_queue_.pop();
  queue_mtx_.unlock();

  // Increase the loaded_counter to record the number of loaded pcd
  ++loaded_counter_;

  return true;
}

template <typename PointT>
bool MapLoader<PointT>::loadManagerThread()
{
  for (auto & it : pcd_to_add_) {
    // Get a free tid
    int idle_tid = get_idle_tid();

    thread_futs_[idle_tid] = std::async(
      std::launch::async, &MapLoader<PointT>::loadThread, this, std::cref(it.first),
      std::cref(it.second));
  }

  return true;
}

template <typename PointT>
void MapLoader<PointT>::parallel_load_clear()
{
  sync();

  output_queue_ = std::queue<std::pair<std::string, PclCloudPtr>>();
  pcd_to_add_.clear();
  pcd_to_remove_.clear();
  loaded_counter_ = 0;
}

template class MapLoader<pcl::PointXYZ>;
template class MapLoader<pcl::PointXYZI>;

}  // namespace loc
