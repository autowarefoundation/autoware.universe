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
/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#include "pointcloud_preprocessor/ground_filter/ray_ground_filter_nodelet.hpp"

#include "pcl_ros/transforms.hpp"

namespace pointcloud_preprocessor
{
RayGroundFilterComponent::RayGroundFilterComponent(const rclcpp::NodeOptions & options)
: Filter("RayGroundFilter", options)
{
  // set initial parameters
  {
    grid_width_ = 1000;
    grid_height_ = 1000;
    grid_precision_ = 0.2;
    ray_ground_filter::generateColors(colors_, color_num_);

    base_frame_ = declare_parameter("base_frame", "base_link");
    general_max_slope_ = declare_parameter("general_max_slope", 8.0);
    local_max_slope_ = declare_parameter("local_max_slope", 6.0);
    radial_divider_angle_ = declare_parameter("radial_divider_angle", 0.08);
    min_height_threshold_ = declare_parameter("min_height_threshold", 0.15);
    concentric_divider_distance_ = declare_parameter("concentric_divider_distance", 0.0);
    reclass_distance_threshold_ = declare_parameter("reclass_distance_threshold", 0.1);
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RayGroundFilterComponent::paramCallback, this, _1));
}

bool RayGroundFilterComponent::TransformPointCloud(
  const std::string & in_target_frame,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud_ptr,
  const sensor_msgs::msg::PointCloud2::SharedPtr & out_cloud_ptr)
{
  if (in_target_frame == in_cloud_ptr->header.frame_id) {
    *out_cloud_ptr = *in_cloud_ptr;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      in_target_frame, in_cloud_ptr->header.frame_id, in_cloud_ptr->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return false;
  }
  // tf2::doTransform(*in_cloud_ptr, *out_cloud_ptr, transform_stamped);
  Eigen::Matrix4f mat = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(mat, *in_cloud_ptr, *out_cloud_ptr);
  out_cloud_ptr->header.frame_id = in_target_frame;
  return true;
}

void RayGroundFilterComponent::ConvertXYZIToRTZColor(
  const pcl::PointCloud<PointType_>::Ptr in_cloud, PointCloudXYZRTColor & out_organized_points,
  std::vector<pcl::PointIndices> & out_radial_divided_indices,
  std::vector<PointCloudXYZRTColor> & out_radial_ordered_clouds)
{
  out_organized_points.resize(in_cloud->points.size());
  out_radial_divided_indices.clear();
  out_radial_divided_indices.resize(radial_dividers_num_);
  out_radial_ordered_clouds.resize(radial_dividers_num_);

  for (size_t i = 0; i < in_cloud->points.size(); i++) {
    PointXYZRTColor new_point;
    auto radius = (float)sqrt(
      in_cloud->points[i].x * in_cloud->points[i].x +
      in_cloud->points[i].y * in_cloud->points[i].y);
    auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
    if (theta < 0) {
      theta += 360;
    }
    if (theta >= 360) {
      theta -= 360;
    }
    auto radial_div = (size_t)floor(theta / radial_divider_angle_);
    auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

    new_point.point.x = in_cloud->points[i].x;
    new_point.point.y = in_cloud->points[i].y;
    new_point.point.z = in_cloud->points[i].z;
    // new_point.ring = in_cloud->points[i].ring;
    new_point.radius = radius;
    new_point.theta = theta;
    new_point.radial_div = radial_div;
    new_point.concentric_div = concentric_div;
    new_point.red = (size_t)colors_[new_point.radial_div % color_num_].val[0];
    new_point.green = (size_t)colors_[new_point.radial_div % color_num_].val[1];
    new_point.blue = (size_t)colors_[new_point.radial_div % color_num_].val[2];
    new_point.original_index = i;

    out_organized_points[i] = new_point;

    // radial divisions
    out_radial_divided_indices[radial_div].indices.push_back(i);

    out_radial_ordered_clouds[radial_div].push_back(new_point);

  }  // end for

  // order radial points on each division
#pragma omp for
  for (size_t i = 0; i < radial_dividers_num_; i++) {
    std::sort(
      out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
      [](const PointXYZRTColor & a, const PointXYZRTColor & b) {return a.radius < b.radius;});
  }
}

void RayGroundFilterComponent::ClassifyPointCloud(
  std::vector<PointCloudXYZRTColor> & in_radial_ordered_clouds,
  pcl::PointIndices & out_ground_indices, pcl::PointIndices & out_no_ground_indices)
{
  out_ground_indices.indices.clear();
  out_no_ground_indices.indices.clear();
#pragma omp for
  for (size_t i = 0; i < in_radial_ordered_clouds.size();
    i++)     // sweep through each radial division
  {
    float prev_radius = 0.f;
    float prev_height = 0.f;
    bool prev_ground = false;
    bool current_ground = false;
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size();
      j++)     // loop through each point in the radial div
    {
      float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
      float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
      float current_height = in_radial_ordered_clouds[i][j].point.z;
      float general_height_threshold =
        tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

      // for points which are very close causing the height threshold to be tiny, set a minimum value
      if (height_threshold < min_height_threshold_) {
        height_threshold = min_height_threshold_;
      }
      // only check points which radius is larger than the concentric_divider
      if (points_distance < concentric_divider_distance_) {
        current_ground = prev_ground;
      } else {
        // check current point height against the LOCAL threshold (previous point)
        if (
          current_height <= (prev_height + height_threshold) &&
          current_height >= (prev_height - height_threshold))
        {
          // Check again using general geometry (radius from origin) if previous points wasn't ground
          if (!prev_ground) {
            if (
              current_height <= general_height_threshold &&
              current_height >= -general_height_threshold)
            {
              current_ground = true;
            } else {
              current_ground = false;
            }
          } else {
            current_ground = true;
          }
        } else {
          // check if previous point is too far from previous one, if so classify again
          if (
            points_distance > reclass_distance_threshold_ &&
            (current_height <= height_threshold && current_height >= -height_threshold))
          {
            current_ground = true;
          } else {
            current_ground = false;
          }
        }
      }  // end larger than concentric_divider

      if (current_ground) {
        out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = true;
      } else {
        out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].point.z;
    }
  }
}

// [ROS2-port]: removed
// bool RayGroundFilterComponent::child_init()
// {
//   // Enable the dynamic reconfigure service
//   has_service = true;
//   srv_ = boost::make_shared<
//     dynamic_reconfigure::Server<pointcloud_preprocessor::RayGroundFilterConfig> >(nh);
//   dynamic_reconfigure::Server<pointcloud_preprocessor::RayGroundFilterConfig>::CallbackType f =
//     boost::bind(&RayGroundFilterComponent::config_callback, this, _1, _2);
//   srv_->setCallback(f);
//   return (true);
// }

void RayGroundFilterComponent::ExtractPointsIndices(
  const pcl::PointCloud<PointType_>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<PointType_>::Ptr out_only_indices_cloud_ptr,
  pcl::PointCloud<PointType_>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<PointType_> extract_ground;
  extract_ground.setInputCloud(in_cloud_ptr);
  extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(in_indices));

  extract_ground.setNegative(false);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true);  // true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

void RayGroundFilterComponent::filter(
  const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices, PointCloud2 & output)
{
  boost::mutex::scoped_lock lock(mutex_);

  sensor_msgs::msg::PointCloud2::SharedPtr input_transed_ptr(new sensor_msgs::msg::PointCloud2);
  bool succeeded = TransformPointCloud(base_frame_, input, input_transed_ptr);
  if (!succeeded) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Failed transform from " << base_frame_ << " to " << input->header.frame_id);
    return;
  }

  pcl::PointCloud<PointType_>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<PointType_>);
  pcl::fromROSMsg(*input_transed_ptr, *current_sensor_cloud_ptr);

  PointCloudXYZRTColor organized_points;
  std::vector<pcl::PointIndices> radial_division_indices;
  std::vector<PointCloudXYZRTColor> radial_ordered_clouds;

  radial_dividers_num_ = ceil(360 / radial_divider_angle_);

  ConvertXYZIToRTZColor(
    current_sensor_cloud_ptr, organized_points, radial_division_indices, radial_ordered_clouds);

  pcl::PointIndices ground_indices, no_ground_indices;

  ClassifyPointCloud(radial_ordered_clouds, ground_indices, no_ground_indices);

  pcl::PointCloud<PointType_>::Ptr ground_cloud_ptr(new pcl::PointCloud<PointType_>);
  pcl::PointCloud<PointType_>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<PointType_>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr radials_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  ExtractPointsIndices(
    current_sensor_cloud_ptr, ground_indices, ground_cloud_ptr, no_ground_cloud_ptr);

  sensor_msgs::msg::PointCloud2::SharedPtr no_ground_cloud_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*no_ground_cloud_ptr, *no_ground_cloud_msg_ptr);
  no_ground_cloud_msg_ptr->header = input->header;
  no_ground_cloud_msg_ptr->header.frame_id = base_frame_;
  sensor_msgs::msg::PointCloud2::SharedPtr no_ground_cloud_transed_msg_ptr(
    new sensor_msgs::msg::PointCloud2);
  succeeded =
    TransformPointCloud(base_frame_, no_ground_cloud_msg_ptr, no_ground_cloud_transed_msg_ptr);
  if (!succeeded) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Failed transform from " << base_frame_ << " to " <<
        no_ground_cloud_msg_ptr->header.frame_id);
    return;
  }
  output = *no_ground_cloud_transed_msg_ptr;
}

rcl_interfaces::msg::SetParametersResult RayGroundFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (get_param(p, "base_frame", base_frame_)) {
    RCLCPP_DEBUG(get_logger(), "Setting base_frame to: %s.", base_frame_);
  }
  if (get_param(p, "general_max_slope", general_max_slope_)) {
    RCLCPP_DEBUG(get_logger(), "Setting general_max_slope to: %f.", general_max_slope_);
  }
  if (get_param(p, "local_max_slope", local_max_slope_)) {
    RCLCPP_DEBUG(get_logger(), "Setting local_max_slope to: %f.", local_max_slope_);
  }
  if (get_param(p, "radial_divider_angle", radial_divider_angle_)) {
    RCLCPP_DEBUG(get_logger(), "Setting radial_divider_angle to: %f.", radial_divider_angle_);
  }
  if (get_param(p, "concentric_divider_distance", concentric_divider_distance_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting concentric_divider_distance to: %f.", concentric_divider_distance_);
  }
  if (get_param(p, "min_height_threshold", min_height_threshold_)) {
    RCLCPP_DEBUG(get_logger(), "Setting min_height_threshold_ to: %f.", min_height_threshold_);
  }
  if (get_param(p, "reclass_distance_threshold", reclass_distance_threshold_)) {
    RCLCPP_DEBUG(
      get_logger(), "Setting reclass_distance_threshold to: %f.", reclass_distance_threshold_);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace pointcloud_preprocessor

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RayGroundFilterComponent)
