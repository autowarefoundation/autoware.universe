/*
 * Copyright 2023 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Mingyu Li
 *
 */

#include "traffic_light_multi_camera_fusion/node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace
{

bool isUnknown(const autoware_auto_perception_msgs::msg::TrafficSignal & signal)
{
  return signal.lights.size() == 1 &&
         signal.lights[0].color == autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN &&
         signal.lights[0].shape == autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN;
}

/**
 * @brief Currently the visible score only considers the truncation.
 * If the detection roi is very close to the image boundary, it would be considered as truncated.
 *
 * @param roi       detection roi
 * @param cam_info  camera info
 * @return 0 if traffic light is truncated, otherwise 1
 */
int calVisibleScore(
  const traffic_light::MultiCameraFusion::RoiType & roi,
  const traffic_light::MultiCameraFusion::CamInfoType & cam_info)
{
  uint32_t boundary = 5;
  uint32_t x1 = roi.roi.x_offset;
  uint32_t x2 = roi.roi.x_offset + roi.roi.width;
  uint32_t y1 = roi.roi.y_offset;
  uint32_t y2 = roi.roi.y_offset + roi.roi.height;
  if (
    x1 <= boundary || (cam_info.width - x2) <= boundary || y1 <= boundary ||
    (cam_info.height - y2) <= boundary) {
    return 0;
  } else {
    return 1;
  }
}

int compareRecord(
  const traffic_light::FusionRecord & r1, const traffic_light::FusionRecord & r2,
  const traffic_light::MultiCameraFusion::CamInfoType & cam_info)
{
  /*
  if both records are from the same sensor but different stamp, trust the latest one
  */
  double t1 = rclcpp::Time(r1.header.stamp).seconds();
  double t2 = rclcpp::Time(r2.header.stamp).seconds();
  if (r1.header.frame_id == r2.header.frame_id && std::abs(t1 - t2) >= 1e-3) {
    return t1 < t2 ? -1 : 1;
  }
  bool r1_is_unknown = isUnknown(r1.signal);
  bool r2_is_unknown = isUnknown(r2.signal);
  /*
  if both are unknown, they are of the same priority
  */
  if (r1_is_unknown && r2_is_unknown) {
    return 0;
  } else if (r1_is_unknown ^ r2_is_unknown) {
    /*
    if either is unknown, the unknown is of lower priority
    */
    return r1_is_unknown ? -1 : 1;
  }
  int visible_score_1 = calVisibleScore(r1.roi, cam_info);
  int visible_score_2 = calVisibleScore(r2.roi, cam_info);
  if (visible_score_1 == visible_score_2) {
    int area_1 = r1.roi.roi.width * r1.roi.roi.height;
    int area_2 = r2.roi.roi.width * r2.roi.roi.height;
    if (area_1 < area_2) {
      return -1;
    } else {
      return static_cast<int>(area_1 > area_2);
    }
  } else {
    return visible_score_1 < visible_score_2 ? -1 : 1;
  }
}

}  // namespace

namespace traffic_light
{

MultiCameraFusion::MultiCameraFusion(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_multi_camera_fusion", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  std::vector<std::string> camera_namespaces =
    this->declare_parameter("camera_namespaces", std::vector<std::string>{});
  is_approximate_sync_ = this->declare_parameter<bool>("approximate_sync", false);
  message_lifespan_ = this->declare_parameter<double>("message_lifespan", 0.09);
  perform_group_fusion_ = this->declare_parameter<bool>("perform_group_fusion", false);
  for (const std::string & camera_ns : camera_namespaces) {
    std::string signal_topic = camera_ns + "/traffic_signals";
    std::string roi_topic = camera_ns + "/rois";
    std::string cam_info_topic = camera_ns + "/camera_info";
    roi_subs_.emplace_back(
      new mf::Subscriber<RoiArrayType>(this, roi_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    signal_subs_.emplace_back(new mf::Subscriber<SignalArrayType>(
      this, signal_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    cam_info_subs_.emplace_back(
      new mf::Subscriber<CamInfoType>(this, cam_info_topic, rclcpp::QoS{1}.get_rmw_qos_profile()));
    if (is_approximate_sync_ == false) {
      exact_sync_subs_.emplace_back(new ExactSync(
        ExactSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      exact_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    } else {
      appro_sync_subs_.emplace_back(new ApproSync(
        ApproSyncPolicy(10), *(cam_info_subs_.back()), *(roi_subs_.back()),
        *(signal_subs_.back())));
      appro_sync_subs_.back()->registerCallback(
        std::bind(&MultiCameraFusion::trafficSignalRoiCallback, this, _1, _2, _3));
    }
  }

  if (perform_group_fusion_) {
    map_sub_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
      std::bind(&MultiCameraFusion::mapCallback, this, _1));
  }
  signal_pub_ = create_publisher<SignalArrayType>("~/output/traffic_signals", rclcpp::QoS{1});
}

void MultiCameraFusion::trafficSignalRoiCallback(
  const CamInfoType::ConstSharedPtr cam_info_msg, const RoiArrayType::ConstSharedPtr roi_msg,
  const SignalArrayType::ConstSharedPtr signal_msg)
{
  rclcpp::Time stamp(roi_msg->header.stamp);
  /*
  Insert the received record array to the table.
  Attention should be paied that this record array might not have the newest timestamp
  */
  record_arr_set_.insert(FusionRecordArr{cam_info_msg->header, *roi_msg, *signal_msg});

  std::map<IdType, FusionRecord> fusionedRecordMap;
  multiCameraFusion(*cam_info_msg, fusionedRecordMap);
  if (perform_group_fusion_) {
    groupFusion(*cam_info_msg, fusionedRecordMap);
  }

  SignalArrayType out_msg;
  out_msg.header = signal_msg->header;
  out_msg.signals.reserve(fusionedRecordMap.size());
  for (const auto & p : fusionedRecordMap) {
    out_msg.signals.emplace_back(p.second.signal);
  }
  signal_pub_->publish(out_msg);
}

void MultiCameraFusion::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg)
{
  lanelet::LaneletMapPtr lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();

  lanelet::utils::conversion::fromBinMsg(*input_msg, lanelet_map_ptr);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  for (auto tl_itr = all_lanelet_traffic_lights.begin(); tl_itr != all_lanelet_traffic_lights.end();
       ++tl_itr) {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (const auto & light : lights) {
      trafficLightId2RegulatoryEleId_[light.id()] = tl->id();
    }
  }
}

void MultiCameraFusion::multiCameraFusion(
  const CamInfoType cam_info, std::map<IdType, FusionRecord> & fusionedRecordMap)
{
  fusionedRecordMap.clear();
  /*
  this should not happen. Just in case
  */
  if (record_arr_set_.empty()) {
    return;
  }
  const rclcpp::Time & newest_stamp(record_arr_set_.rbegin()->header.stamp);
  for (auto it = record_arr_set_.begin(); it != record_arr_set_.end();) {
    /*
    remove all old record arrays whose timestamp difference with newest record is larger than
    threshold
    */
    if (
      (newest_stamp - rclcpp::Time(it->header.stamp)) >
      rclcpp::Duration::from_seconds(message_lifespan_)) {
      it = record_arr_set_.erase(it);
    } else {
      /*
      generate fusioned record result with the saved records
      */
      const FusionRecordArr & record_arr = *it;
      for (size_t i = 0; i < record_arr.rois.rois.size(); i++) {
        const RoiType & roi = record_arr.rois.rois[i];
        auto signal_it = std::find_if(
          record_arr.signals.signals.begin(), record_arr.signals.signals.end(),
          [roi](const SignalType & s1) { return roi.id == s1.map_primitive_id; });
        /*
        failed to find corresponding signal. skip it
        */
        if (signal_it == record_arr.signals.signals.end()) {
          continue;
        }
        FusionRecord record{record_arr.header, roi, *signal_it};
        /*
        if this traffic light is not detected yet or can be updated by higher priority record,
        update it
        */
        if (
          fusionedRecordMap.find(roi.id) == fusionedRecordMap.end() ||
          ::compareRecord(record, fusionedRecordMap[roi.id], cam_info) >= 0) {
          fusionedRecordMap[roi.id] = record;
        }
      }
      it++;
    }
  }
}

void MultiCameraFusion::groupFusion(
  const CamInfoType cam_info, std::map<IdType, FusionRecord> & fusionedRecordMap)
{
  /*
  this should not happen. Just in case
  */
  if (perform_group_fusion_ == false) {
    RCLCPP_WARN(get_logger(), "perform_group_fusion_ is set to false. Skip GroupFusion");
    return;
  }
  std::map<IdType, FusionRecord> regEleId2BestRecord;
  for (auto & p : fusionedRecordMap) {
    IdType roi_id = p.second.roi.id;
    /*
    this should not happen
    */
    if (trafficLightId2RegulatoryEleId_.count(roi_id) == 0) {
      RCLCPP_WARN_STREAM(
        get_logger(), "Found Traffic Light Id = " << roi_id << " which is not defined in Map");
    } else {
      /*
      keep the best record for every regulatory element id
      */
      IdType reg_ele_id = trafficLightId2RegulatoryEleId_[p.second.roi.id];
      if (
        regEleId2BestRecord.count(reg_ele_id) == 0 ||
        ::compareRecord(p.second, regEleId2BestRecord[reg_ele_id], cam_info) >= 0) {
        regEleId2BestRecord[reg_ele_id] = p.second;
      }
    }
  }
  /*
  replace the resul with the best record of its group
  */
  for (auto & p : fusionedRecordMap) {
    if (trafficLightId2RegulatoryEleId_.count(p.second.roi.id) != 0) {
      IdType reg_ele_id = trafficLightId2RegulatoryEleId_[p.second.roi.id];
      p.second.signal.lights = regEleId2BestRecord[reg_ele_id].signal.lights;
    }
  }
}

}  // namespace traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::MultiCameraFusion)
