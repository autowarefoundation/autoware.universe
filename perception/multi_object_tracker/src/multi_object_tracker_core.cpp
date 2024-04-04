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
//
//

#include <boost/optional.hpp>

#include <glog/logging.h>
#include <tf2_ros/create_timer_interface.h>
#include <tf2_ros/create_timer_ros.h>

#include <iterator>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include "multi_object_tracker/multi_object_tracker_core.hpp"
#include "multi_object_tracker/utils/utils.hpp"
// #include "object_recognition_utils/object_recognition_utils.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp_components/register_node_macro.hpp>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;

namespace
{
boost::optional<geometry_msgs::msg::Transform> getTransformAnonymous(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    // check if the frames are ready
    std::string errstr;  // This argument prevents error msg from being displayed in the terminal.
    if (!tf_buffer.canTransform(
          target_frame_id, source_frame_id, tf2::TimePointZero, tf2::Duration::zero(), &errstr)) {
      return boost::none;
    }

    geometry_msgs::msg::TransformStamped self_transform_stamped;
    self_transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return self_transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("multi_object_tracker"), ex.what());
    return boost::none;
  }
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("multi_object_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  last_published_time_(this->now())
{
  // glog for debug
  google::InitGoogleLogging("multi_object_tracker");
  google::InstallFailureSignalHandler();

  // Create publishers and subscribers
  detected_object_sub_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "input", rclcpp::QoS{1},
    std::bind(&MultiObjectTracker::onMeasurement, this, std::placeholders::_1));
  tracked_objects_pub_ =
    create_publisher<autoware_auto_perception_msgs::msg::TrackedObjects>("output", rclcpp::QoS{1});

  // Parameters
  double publish_rate = declare_parameter<double>("publish_rate");  // [hz]
  world_frame_id_ = declare_parameter<std::string>("world_frame_id");
  bool enable_delay_compensation{declare_parameter<bool>("enable_delay_compensation")};

  // Create tf timer
  auto cti = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(cti);

  // Create ROS time based timer.
  // If the delay compensation is enabled, the timer is used to publish the output at the correct
  // time.
  if (enable_delay_compensation) {
    publisher_period_ = 1.0 / publish_rate;    // [s]
    constexpr double timer_multiplier = 20.0;  // 20 times frequent for publish timing check
    const auto timer_period = rclcpp::Rate(publish_rate * timer_multiplier).period();
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), timer_period, std::bind(&MultiObjectTracker::onTimer, this));
  }

  // Initialize processor
  {
    std::map<std::uint8_t, std::string> tracker_map;
    tracker_map.insert(
      std::make_pair(Label::CAR, this->declare_parameter<std::string>("car_tracker")));
    tracker_map.insert(
      std::make_pair(Label::TRUCK, this->declare_parameter<std::string>("truck_tracker")));
    tracker_map.insert(
      std::make_pair(Label::BUS, this->declare_parameter<std::string>("bus_tracker")));
    tracker_map.insert(
      std::make_pair(Label::TRAILER, this->declare_parameter<std::string>("trailer_tracker")));
    tracker_map.insert(std::make_pair(
      Label::PEDESTRIAN, this->declare_parameter<std::string>("pedestrian_tracker")));
    tracker_map.insert(
      std::make_pair(Label::BICYCLE, this->declare_parameter<std::string>("bicycle_tracker")));
    tracker_map.insert(std::make_pair(
      Label::MOTORCYCLE, this->declare_parameter<std::string>("motorcycle_tracker")));

    processor_ = std::make_unique<TrackerProcessor>(tracker_map);
  }

  // Data association initialization
  {
    const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
    const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
    const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
    const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
    const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
    const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
    const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");

    data_association_ = std::make_unique<DataAssociation>(
      can_assign_matrix, max_dist_matrix, max_area_matrix, min_area_matrix, max_rad_matrix,
      min_iou_matrix);
  }

  // Debugger
  debugger_ = std::make_unique<TrackerDebugger>(*this);
  published_time_publisher_ = std::make_unique<tier4_autoware_utils::PublishedTimePublisher>(this);
}

void MultiObjectTracker::onMeasurement(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr input_objects_msg)
{
  // Get the time of the measurement
  const rclcpp::Time measurement_time =
    rclcpp::Time(input_objects_msg->header.stamp, this->now().get_clock_type());

  /* keep the latest input stamp and check transform*/
  debugger_->startMeasurementTime(this->now(), measurement_time);
  const auto self_transform =
    getTransformAnonymous(tf_buffer_, "base_link", world_frame_id_, measurement_time);
  if (!self_transform) {
    return;
  }
  /* transform to world coordinate */
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!object_recognition_utils::transformObjects(
        *input_objects_msg, world_frame_id_, tf_buffer_, transformed_objects)) {
    return;
  }

  ////// Tracker Process
  //// Associate and update
  /* prediction */
  processor_->predict(measurement_time);
  /* association */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  {
    const auto & list_tracker = processor_->getListTracker();
    const auto & detected_objects = transformed_objects;
    // global nearest neighbor
    Eigen::MatrixXd score_matrix = data_association_->calcScoreMatrix(
      detected_objects, list_tracker);  // row : tracker, col : measurement
    data_association_->assign(score_matrix, direct_assignment, reverse_assignment);
  }
  /* tracker update */
  processor_->update(transformed_objects, *self_transform, direct_assignment);

  //// Tracker management
  // if the tracker is old, delete it
  processor_->checkTrackerLifeCycle(measurement_time);
  // if the tracker is too close, delete it
  processor_->sanitizeTracker(measurement_time);

  //// Spawn new tracker
  processor_->spawn(transformed_objects, *self_transform, reverse_assignment);

  // debugger time
  debugger_->endMeasurementTime(this->now());

  // Publish objects if the timer is not enabled
  if (publish_timer_ == nullptr) {
    // Publish if the delay compensation is disabled
    const auto & list_tracker = processor_->getListTracker();
    publish(measurement_time, list_tracker);
  } else {
    // Publish if the next publish time is close
    const double minimum_publish_interval = publisher_period_ * 0.70;  // 70% of the period
    if ((this->now() - last_published_time_).seconds() > minimum_publish_interval) {
      checkAndPublish(this->now());
    }
  }
}

void MultiObjectTracker::onTimer()
{
  const rclcpp::Time current_time = this->now();
  // check the publish period
  const auto elapsed_time = (current_time - last_published_time_).seconds();
  // if the elapsed time is over the period, publish objects with prediction
  constexpr double maximum_latency_ratio = 1.11;  // 11% margin
  const double maximum_publish_latency = publisher_period_ * maximum_latency_ratio;
  if (elapsed_time > maximum_publish_latency) {
    checkAndPublish(current_time);
  }
}

void MultiObjectTracker::checkAndPublish(const rclcpp::Time & time)
{
  /* life cycle check */
  // if the tracker is old, delete it
  processor_->checkTrackerLifeCycle(time);

  /* sanitize trackers */
  // if the tracker is too close, delete it
  processor_->sanitizeTracker(time);

  // Publish
  const auto & list_tracker = processor_->getListTracker();
  publish(time, list_tracker);

  // Update last published time
  last_published_time_ = this->now();
}

inline bool MultiObjectTracker::shouldTrackerPublish(
  const std::shared_ptr<const Tracker> tracker) const
{
  constexpr int measurement_count_threshold = 3;
  if (tracker->getTotalMeasurementCount() < measurement_count_threshold) {
    return false;
  }
  return true;
}

void MultiObjectTracker::publish(
  const rclcpp::Time & time, const std::list<std::shared_ptr<Tracker>> & list_tracker) const
{
  debugger_->startPublishTime(this->now());
  const auto subscriber_count = tracked_objects_pub_->get_subscription_count() +
                                tracked_objects_pub_->get_intra_process_subscription_count();
  if (subscriber_count < 1) {
    return;
  }
  // Create output msg
  autoware_auto_perception_msgs::msg::TrackedObjects output_msg, tentative_objects_msg;
  output_msg.header.frame_id = world_frame_id_;
  output_msg.header.stamp = time;
  tentative_objects_msg.header = output_msg.header;

  for (auto itr = list_tracker.begin(); itr != list_tracker.end(); ++itr) {
    if (!shouldTrackerPublish(*itr)) {  // for debug purpose
      autoware_auto_perception_msgs::msg::TrackedObject object;
      if (!(*itr)->getTrackedObject(time, object)) continue;
      tentative_objects_msg.objects.push_back(object);
      continue;
    }
    autoware_auto_perception_msgs::msg::TrackedObject object;
    if (!(*itr)->getTrackedObject(time, object)) continue;
    output_msg.objects.push_back(object);
  }

  // Publish
  tracked_objects_pub_->publish(output_msg);
  published_time_publisher_->publish_if_subscribed(tracked_objects_pub_, output_msg.header.stamp);

  // Debugger Publish if enabled
  debugger_->endPublishTime(this->now(), time);
  debugger_->publishTentativeObjects(tentative_objects_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(MultiObjectTracker)
