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

#include <pthread.h>
#include <semaphore.h>

#include <condition_variable>
#include <list>
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

  // public methods for calling from detached pthreads. pthreads are detached from the
  // "workingThread"
  void pThreadJob(int rank);

  // semaphores to control the starting and finishing of phthread tasks
  sem_t * starting_semaphores;
  sem_t * ending_semaphores;

private:
  void processMessage(PredictedObjects::ConstSharedPtr msg) override;

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
  // num_threads<=1;
  std::vector<visualization_msgs::msg::Marker::SharedPtr> createMarkers(
    PredictedObjects::ConstSharedPtr msg);
  // Working thread is detached from the main to control the flow of messages
  void workerThread();

  void update(float wall_dt, float ros_dt) override;

  std::vector<visualization_msgs::msg::Marker::SharedPtr> tackle_one_object(
    PredictedObjects::ConstSharedPtr _msg, int index,
    std::vector<visualization_msgs::msg::Marker::SharedPtr> _markers);

  // For pthreads to push to the same "tmp_maker" vector
  void push_tmp_markers(std::vector<visualization_msgs::msg::Marker::SharedPtr> marker_ptrs);

  std::unordered_map<boost::uuids::uuid, int32_t, boost::hash<boost::uuids::uuid>> id_map;
  // std::unordered_map<boost::uuids::uuid, int32_t> id_map;
  std::list<int32_t> unused_marker_ids;
  int32_t marker_id = 0;
  const int32_t PATH_ID_CONSTANT = 1e3;

  // msg: latest message, passed from the main callback to workThread;
  PredictedObjects::ConstSharedPtr msg;

  // tmp_msg: latest message, maintained inside working threads and consumed by pthreads
  PredictedObjects::ConstSharedPtr tmp_msg;
  bool consumed{false};

  // mutex: mutex for accessing this->msg, handle conflicts between main process and the working
  // thread;
  std::mutex mutex;

  // tmp_mutex: mutex for accessing this->tmp_markers, handle conflicts between pthreads;
  pthread_mutex_t tmp_marker_mutex;

  // num_threads: number of threads going to be used, can be modifed on the fly in RVIZ parameters;
  int num_threads;
  // max_num_threads: number of threads to be created, hard-coded to be 8;
  int max_num_threads;
  // condition: condition variable for working thread to wait for main callback to get this->msg;
  std::condition_variable condition;
  // marker: markers to be published, created by working thread and consumed by the "update" of main
  // process;
  std::vector<visualization_msgs::msg::Marker::SharedPtr> markers;
  // tmp_markers: marker vectors being created by the pthreads, will be passed to marker when
  // finished.
  std::vector<visualization_msgs::msg::Marker::SharedPtr> tmp_markers;
  // existing_marker_ids: marker ids that were maintained in the last frame.
  std::set<rviz_default_plugins::displays::MarkerID> existing_marker_ids;

  rviz_common::properties::IntProperty * num_threads_property;
};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // OBJECT_DETECTION__PREDICTED_OBJECTS_DISPLAY_HPP_
