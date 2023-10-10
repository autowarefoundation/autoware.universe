// Copyright 2020 Tier IV, Inc.
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
#ifndef OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_
#define OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_

#include <object_detection/object_polygon_display_base.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <semaphore.h>

#include <condition_variable>
#include <list>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
/// \brief Class defining rviz plugin to visualize PredictedObjects
class AUTOWARE_AUTO_PERCEPTION_RVIZ_PLUGIN_PUBLIC PredictedObjectsDisplay
: public ObjectPolygonDisplayBase<autoware_auto_perception_msgs::msg::PredictedObjects>
{
  Q_OBJECT

public:
  using PredictedObjects = autoware_auto_perception_msgs::msg::PredictedObjects;

  PredictedObjectsDisplay();
  ~PredictedObjectsDisplay()
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      should_terminate = true;
    }
    mutex_condition.notify_all();
    for (std::thread & active_thread : threads) {
      active_thread.join();
    }
    threads.clear();

    for (int ii = 0; ii < max_num_threads; ++ii) {
      sem_close(&task_completed_semaphores[ii]);
    }
    ObjectPolygonDisplayBase<autoware_auto_perception_msgs::msg::PredictedObjects>::
      ~ObjectPolygonDisplayBase<autoware_auto_perception_msgs::msg::PredictedObjects>();
  }

private:
  void processMessage(PredictedObjects::ConstSharedPtr msg) override;

  void queueJob(std::function<void()> & job)
  {
    {
      std::unique_lock<std::mutex> lock(queue_mutex);
      jobs.push(job);
    }
    mutex_condition.notify_one();
  }

  boost::uuids::uuid to_boost_uuid(const unique_identifier_msgs::msg::UUID & uuid_msg)
  {
    const std::string uuid_str = uuid_to_string(uuid_msg);
    boost::uuids::string_generator gen;
    boost::uuids::uuid uuid = gen(uuid_str);
    return uuid;
  }

  void update_id_map(const PredictedObjects::ConstSharedPtr & msg)
  {
    std::vector<boost::uuids::uuid> new_uuids;
    std::vector<boost::uuids::uuid> tracked_uuids;
    new_uuids.reserve(msg->objects.size());
    tracked_uuids.reserve(msg->objects.size());
    for (const auto & object : msg->objects) {
      const auto uuid = to_boost_uuid(object.object_id);
      ((id_map.find(uuid) != id_map.end()) ? tracked_uuids : new_uuids).push_back(uuid);
    }

    auto itr = id_map.begin();
    while (itr != id_map.end()) {
      if (
        std::find(tracked_uuids.begin(), tracked_uuids.end(), itr->first) == tracked_uuids.end()) {
        unused_marker_ids.push_back(itr->second);
        itr = id_map.erase(itr);
      } else {
        ++itr;
      }
    }

    for (const auto & new_uuid : new_uuids) {
      if (unused_marker_ids.empty()) {
        id_map.emplace(new_uuid, marker_id);
        marker_id++;
      } else {
        id_map.emplace(new_uuid, unused_marker_ids.front());
        unused_marker_ids.pop_front();
      }
    }
  }

  int32_t uuid_to_marker_id(const unique_identifier_msgs::msg::UUID & uuid_msg)
  {
    auto uuid = to_boost_uuid(uuid_msg);
    return id_map.at(uuid);
  }

  // createMarkers: create markers from the message with the single working process; activated when
  // num_threads=1;
  std::vector<visualization_msgs::msg::Marker::SharedPtr> createMarkers(
    PredictedObjects::ConstSharedPtr msg);
  // Working thread is detached from the main to control the flow of messages
  void workerThread();
  // parallelizedCreateMarkerWorkerThread: create markers from the message with multiple workers
  void parallelizedCreateMarkerWorkerThread(int rank);

  void messageProcessorThreadJob();

  void update(float wall_dt, float ros_dt) override;

  std::vector<visualization_msgs::msg::Marker::SharedPtr> tackle_one_object(
    PredictedObjects::ConstSharedPtr msg, int index,
    std::vector<visualization_msgs::msg::Marker::SharedPtr> markers);

  void push_tmp_markers(std::vector<visualization_msgs::msg::Marker::SharedPtr> marker_ptrs);

  std::unordered_map<boost::uuids::uuid, int32_t, boost::hash<boost::uuids::uuid>> id_map;
  // std::unordered_map<boost::uuids::uuid, int32_t> id_map;
  std::list<int32_t> unused_marker_ids;
  int32_t marker_id = 0;
  const int32_t PATH_ID_CONSTANT = 1e3;

  // msg: latest message, passed from the main callback to workThread;
  PredictedObjects::ConstSharedPtr msg;

  // tmp_msg: latest message, maintained inside working threads and consumed by parallelized workers
  PredictedObjects::ConstSharedPtr tmp_msg;
  bool consumed{false};

  // mutex: mutex for accessing this->msg, handle conflicts between main process and the working
  // thread;
  std::mutex mutex;

  // tmp_marker_mutex: mutex for accessing this->tmp_markers, handle conflicts between parallelized
  // workers;
  std::mutex tmp_marker_mutex;

  // num_threads: number of threads going to be used, can be modifed on the fly in RViz parameters;
  int num_threads;
  // max_num_threads: number of threads to be created, hard-coded to be 8;
  int max_num_threads;

  bool should_terminate{false};
  std::mutex queue_mutex;
  std::condition_variable mutex_condition;
  std::vector<std::thread> threads;
  std::queue<std::function<void()>> jobs;

  // semaphores showing the completion of marker creation in each thread
  sem_t * task_completed_semaphores;

  // condition: condition variable for working thread to wait for main callback to get this->msg;
  std::condition_variable condition;
  // markers: markers to be published, created by working thread and consumed by the "update" of
  // main process;
  std::vector<visualization_msgs::msg::Marker::SharedPtr> markers;
  // tmp_markers: marker vectors being created by the parallelized workers, will be passed to marker
  // when finished.
  std::vector<visualization_msgs::msg::Marker::SharedPtr> tmp_markers;
  // existing_marker_ids: marker ids that were maintained in the last frame.
  std::set<rviz_default_plugins::displays::MarkerID> existing_marker_ids;

  rviz_common::properties::IntProperty * num_threads_property;
};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_
