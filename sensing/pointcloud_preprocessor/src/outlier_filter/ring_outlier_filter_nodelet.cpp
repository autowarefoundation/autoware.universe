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

#include "pointcloud_preprocessor/outlier_filter/ring_outlier_filter_nodelet.hpp"

#include "pointcloud_preprocessor/utility/utilities.hpp"

#include <range/v3/view/chunk.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace pointcloud_preprocessor
{
RingOutlierFilterComponent::RingOutlierFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RingOutlierFilter", options)
{
  // initialize debug tool
  {
    using tier4_autoware_utils::DebugPublisher;
    using tier4_autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<DebugPublisher>(this, "ring_outlier_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // set initial parameters
  {
    distance_ratio_ = static_cast<double>(declare_parameter("distance_ratio", 1.03));
    object_length_threshold_ =
      static_cast<double>(declare_parameter("object_length_threshold", 0.1));
    num_points_threshold_ = static_cast<int>(declare_parameter("num_points_threshold", 4));
    max_rings_num_ = static_cast<uint16_t>(declare_parameter("max_rings_num", 128));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RingOutlierFilterComponent::paramCallback, this, _1));
}

// TODO(sykwer): Temporary Implementation: Rename this function to `filter()` when all the filter
// nodes conform to new API. Then delete the old `filter()` defined below.
void RingOutlierFilterComponent::faster_filter(
  const PointCloud2ConstPtr & input, const IndicesPtr & unused_indices, PointCloud2 & output,
  const TransformInfo & transform_info)
{
  std::scoped_lock lock(mutex_);
  if (unused_indices) {
    RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
  }
  stop_watch_ptr_->toc("processing_time", true);

  // point field extraction helpers

  // FIXME(VRichardJP) Everything would be simpler if Autoware enforced strict PointCloud2 format

  auto maybe_intensity_offset = utils::getIntensityOffset(input->fields);
  if (!maybe_intensity_offset) {
    RCLCPP_ERROR(get_logger(), "input cloud does not contain 'intensity' data field");
    return;
  }
  const auto intensity_offset = *maybe_intensity_offset;

  auto maybe_ring_offset = utils::getRingOffset(input->fields);
  if (!maybe_ring_offset) {
    RCLCPP_ERROR(get_logger(), "input cloud does not contain 'ring' data field");
    return;
  }
  const auto ring_offset = *maybe_ring_offset;

  auto maybe_azimuth_offset = utils::getAzimuthOffset(input->fields);
  if (!maybe_azimuth_offset) {
    RCLCPP_ERROR(get_logger(), "input cloud does not contain 'azimuth' data field");
    return;
  }
  const auto azimuth_offset = *maybe_azimuth_offset;

  auto maybe_distance_offset = utils::getDistanceOffset(input->fields);
  if (!maybe_distance_offset) {
    RCLCPP_ERROR(get_logger(), "input cloud does not contain 'distance' data field");
    return;
  }
  const auto distance_offset = *maybe_distance_offset;

  // extract intensity from point raw data
  auto getPointIntensity = [=](const unsigned char * raw_p) {
    float intensity{};
    std::memcpy(&intensity, &raw_p[intensity_offset], sizeof(intensity));
    return intensity;
  };

  // extract ring from point raw data
  auto getPointRing = [=](const unsigned char * raw_p) {
    uint16_t ring{};
    std::memcpy(&ring, &raw_p[ring_offset], sizeof(ring));
    return ring;
  };

  // extract azimuth from point raw data
  auto getPointAzimuth = [=](const unsigned char * raw_p) {
    float azimuth{};
    std::memcpy(&azimuth, &raw_p[azimuth_offset], sizeof(azimuth));
    return azimuth;
  };

  // extract distance from point raw data
  auto getPointDistance = [=](const unsigned char * raw_p) {
    float distance{};
    std::memcpy(&distance, &raw_p[distance_offset], sizeof(distance));
    return distance;
  };

  // The initial implementation of ring outlier filter looked like this:
  //   1. Iterate over the input cloud and group point indices by ring
  //   2. For each ring:
  //   2.1. iterate over the ring points, and group points belonging to the same "walk"
  //   2.2. when a walk is closed, copy indexed points to the output cloud if the walk is long
  //   enough.
  //
  // Because LIDAR data is naturally "azimut-major order" and not "ring-major order", such
  // implementation is not cache friendly at all, and has negative impact on all the other nodes.
  //
  // To tackle this issue, the algorithm has been rewritten so that points would be accessed in
  // order. To do so, all rings are being processing simultaneously instead of separately. The
  // overall logic is still the same.

  // ad-hoc struct to store finished walks information (for isCluster())
  struct WalkInfo
  {
    int num_points;
    float first_point_distance;
    float first_point_azimuth;
    float last_point_distance;
    float last_point_azimuth;
  };

  // ad-hoc struct to keep track of each ring current walk
  struct RingWalkInfo
  {
    size_t walk_idx;      // index of associated walk in walks vector
    float prev_azimuth;   // azimuth of previous point on the ring
    float prev_distance;  // distance of previous point on the ring
  };

  // helper functions

  // check if walk is a valid cluster
  const float object_length_threshold2 = object_length_threshold_ * object_length_threshold_;
  auto isCluster = [=](const WalkInfo & walk_info) {
    // A cluster is a walk which has many points or is long enough
    if (walk_info.num_points > num_points_threshold_) return true;

    // Using the law of cosines, the distance between 2 points can be written as:
    //   |p2-p1|^2 = d1^2 + d2^2 - 2*d1*d2*cos(a)
    // where d1 and d2 are the distance attribute of p1 and p2, and 'a' the azimuth diff between
    // the 2 points.
    const float dist2 =
      walk_info.first_point_distance * walk_info.first_point_distance +
      walk_info.last_point_distance * walk_info.last_point_distance -
      2 * walk_info.first_point_distance * walk_info.last_point_distance *
        std::cos((walk_info.last_point_azimuth - walk_info.first_point_azimuth) * (M_PI / 18000.0));
    return dist2 > object_length_threshold2;
  };

  // check if 2 points belong to the same walk
  auto isSameWalk =
    [=](float curr_distance, float curr_azimuth, float prev_distance, float prev_azimuth) {
      float azimuth_diff = curr_azimuth - prev_azimuth;
      azimuth_diff = azimuth_diff < 0.f ? azimuth_diff + 36000.f : azimuth_diff;
      return std::max(curr_distance, prev_distance) <
               std::min(curr_distance, prev_distance) * distance_ratio_ &&
             azimuth_diff < 100.f;
    };

  // tmp vectors to keep track of walk/ring state while processing points in order (cache efficient)
  std::vector<WalkInfo> walks;                // all walks
  std::vector<RingWalkInfo> rings_curr_walk;  // each ring curr walk info
  std::vector<size_t> rings_first_walk_idx;   // each ring very first walk idx
  std::vector<size_t> points_walk_idx;        // each point walk idx

  walks.reserve(max_rings_num_);
  // initialize each ring with empty walk
  rings_curr_walk.resize(max_rings_num_, RingWalkInfo{-1U, 0., 0.});
  rings_first_walk_idx.resize(max_rings_num_, -1U);
  // points are initially associated to no walk
  points_walk_idx.resize(input->width * input->height, -1U);

  int invalid_ring_count = 0;

  // Build walks and classify points
  for (const auto & [raw_p, point_walk_idx] :
       ranges::views::zip(input->data | ranges::views::chunk(input->point_step), points_walk_idx)) {
    const uint16_t ring_idx = getPointRing(raw_p.data());
    const float curr_azimuth = getPointAzimuth(raw_p.data());
    const float curr_distance = getPointDistance(raw_p.data());

    if (ring_idx >= max_rings_num_) {
      // Either the data is corrupted or max_rings_num_ is not set correctly
      // Note: point_walk_idx == -1 so the point will be filtered out
      invalid_ring_count += 1;
      continue;
    }

    auto & ring = rings_curr_walk[ring_idx];
    if (ring.walk_idx == -1U) {
      // first walk ever for this ring
      walks.push_back(WalkInfo{1, curr_distance, curr_azimuth, curr_distance, curr_azimuth});
      const auto walk_idx = walks.size() - 1;
      point_walk_idx = walk_idx;
      rings_first_walk_idx[ring_idx] = walk_idx;
      ring.walk_idx = walk_idx;
      ring.prev_azimuth = curr_azimuth;
      ring.prev_distance = curr_distance;
      continue;
    }

    auto & walk = walks[ring.walk_idx];
    if (isSameWalk(curr_distance, curr_azimuth, ring.prev_distance, ring.prev_azimuth)) {
      // current point is part of previous walk
      walk.num_points += 1;
      walk.last_point_distance = curr_distance;
      walk.last_point_azimuth = curr_azimuth;
      point_walk_idx = ring.walk_idx;
    } else {
      // previous walk is finished, start a new one
      walks.push_back(WalkInfo{1, curr_distance, curr_azimuth, curr_distance, curr_azimuth});
      point_walk_idx = walks.size() - 1;
      ring.walk_idx = walks.size() - 1;
    }

    ring.prev_azimuth = curr_azimuth;
    ring.prev_distance = curr_distance;
  }

  // So far, we have processed ring points as if rings were not circular. Of course, the last and
  // first points of a ring could totally be part of the same walk. When such thing happens, we need
  // to merge the two walks
  for (const auto & [first_walk_idx, last_walk_idx] : ranges::views::zip(
         rings_first_walk_idx, rings_curr_walk | ranges::views::transform([](const auto & ring) {
                                 return ring.walk_idx;
                               }))) {
    if (first_walk_idx == -1U || last_walk_idx == -1U) {
      continue;
    }
    auto & first_walk = walks[first_walk_idx];
    auto & last_walk = walks[last_walk_idx];
    // check if the two walks are connected
    if (isSameWalk(
          first_walk.first_point_distance, first_walk.first_point_azimuth,
          last_walk.last_point_distance, last_walk.last_point_azimuth)) {
      // merge
      first_walk.first_point_distance = last_walk.first_point_distance;
      first_walk.first_point_azimuth = last_walk.first_point_azimuth;
      last_walk.last_point_distance = first_walk.last_point_distance;
      last_walk.last_point_azimuth = first_walk.last_point_azimuth;
    }
  }

  // segregate walks by cluster (for fast lookup)
  std::vector<bool> walk_is_cluster;
  walk_is_cluster.reserve(walks.size());
  for (const auto & walk_info : walks) {
    walk_is_cluster.push_back(isCluster(walk_info));
  }

  // finally copy points
  output.point_step = sizeof(PointXYZI);
  output.data.resize(output.point_step * input->width * input->height);
  size_t output_size = 0;
  if (transform_info.need_transform) {
    for (const auto & [raw_p, point_walk_idx] : ranges::views::zip(
           input->data | ranges::views::chunk(input->point_step), points_walk_idx)) {
      if (point_walk_idx == -1U || !walk_is_cluster[point_walk_idx]) {
        continue;
      }

      // assume binary representation of input point is compatible with PointXYZ*
      PointXYZI out_point;
      std::memcpy(&out_point, raw_p.data(), sizeof(PointXYZI));

      Eigen::Vector4f p(out_point.x, out_point.y, out_point.z, 1);
      p = transform_info.eigen_transform * p;
      out_point.x = p[0];
      out_point.y = p[1];
      out_point.z = p[2];
      // FIXME(VRichardJP) tmp fix because input data does not have the same layout than PointXYZI
      out_point.intensity = getPointIntensity(raw_p.data());

      std::memcpy(&output.data[output_size], &out_point, sizeof(PointXYZI));
      output_size += sizeof(PointXYZI);
    }
  } else {
    for (const auto & [raw_p, point_walk_idx] : ranges::views::zip(
           input->data | ranges::views::chunk(input->point_step), points_walk_idx)) {
      if (point_walk_idx == -1U || !walk_is_cluster[point_walk_idx]) {
        continue;
      }

#if 0
      // assume binary representation of input point is compatible with PointXYZI
      std::memcpy(&output.data[output_size], raw_p.data(), sizeof(PointXYZI));
#else
      // FIXME(VRichardJP) tmp fix because input data does not have the same layout than PointXYZI
      PointXYZI out_point;
      std::memcpy(&out_point, raw_p.data(), sizeof(PointXYZI));
      out_point.intensity = getPointIntensity(raw_p.data());
      std::memcpy(&output.data[output_size], &out_point, sizeof(PointXYZI));
#endif

      output_size += sizeof(PointXYZI);
    }
  }
  output.data.resize(output_size);

  // Note that `input->header.frame_id` is data before converted when `transform_info.need_transform
  // == true`
  output.header.frame_id = !tf_input_frame_.empty() ? tf_input_frame_ : tf_input_orig_frame_;

  output.height = 1;
  output.width = static_cast<uint32_t>(output.data.size() / output.height / output.point_step);
  output.is_bigendian = input->is_bigendian;
  output.is_dense = input->is_dense;

  // set fields
  sensor_msgs::PointCloud2Modifier pcd_modifier(output);
  constexpr int num_fields = 4;
  pcd_modifier.setPointCloud2Fields(
    num_fields, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

  if (invalid_ring_count > 0) {
    RCLCPP_WARN(
      get_logger(), "%d points had ring index over max_rings_num (%d) and have been ignored.",
      invalid_ring_count, max_rings_num_);
  }

  // add processing time for debug
  if (debug_publisher_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

// TODO(sykwer): Temporary Implementation: Delete this function definition when all the filter nodes
// conform to new API
void RingOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  (void)input;
  (void)indices;
  (void)output;
}

rcl_interfaces::msg::SetParametersResult RingOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "distance_ratio", distance_ratio_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new distance ratio to: %f.", distance_ratio_);
  }
  if (get_param(p, "object_length_threshold", object_length_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting new object length threshold to: %f.", object_length_threshold_);
  }
  if (get_param(p, "num_points_threshold", num_points_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new num_points_threshold to: %d.", num_points_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RingOutlierFilterComponent)
