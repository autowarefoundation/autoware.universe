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

#include "pointcloud_preprocessor/outlier_filter/radius_search_2d_outlier_filter_nodelet.hpp"

namespace pointcloud_preprocessor
{
RadiusSearch2DOutlierFilterComponent::RadiusSearch2DOutlierFilterComponent(
  const rclcpp::NodeOptions & options)
: Filter("RadiusSearch2DOutlierFilter", options)
{
  // set initial parameters
  {
    min_neighbors_ = static_cast<size_t>(declare_parameter("min_neighbors", 5));
    search_radius_ = static_cast<double>(declare_parameter("search_radius", 0.2));
  }

  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&RadiusSearch2DOutlierFilterComponent::paramCallback, this, _1));
}

void RadiusSearch2DOutlierFilterComponent::filter(
  const PointCloud2ConstPtr & input, PointCloud2 & output)
{
  size_t point_size input->points.size();
  std::vector<size_t> neighbors(point_size, 0);
  for (size_t i = 0; i < point_size; i++) {
    for (size_t j = i + 1; j < point_size; j++) {
      size_t square_distance =
        (input->points[i].x - input->points[j].x) * (input->points[i].x - input->points[j].x) + 
        (input->points[i].y - input->points[j].y) * (input->points[i].y - input->points[j].y);
      if (square_distance <= search_radius_ * search_radius_) {
        neighbors[i]++;
        neighbors[j]++;
      }
    }

    if (neighbors[i] >= min_neighbors_) {
      output->points.push_back(input[i]);
    }
  }

  output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult RadiusSearch2DOutlierFilterComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & p)
{
  std::scoped_lock lock(mutex_);

  if (get_param(p, "min_neighbors", min_neighbors_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new min neighbors to: %zu.", min_neighbors_);
  }
  if (get_param(p, "search_radius", search_radius_)) {
    RCLCPP_DEBUG(get_logger(), "Setting new search radius to: %f.", search_radius_);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::RadiusSearch2DOutlierFilterComponent)
