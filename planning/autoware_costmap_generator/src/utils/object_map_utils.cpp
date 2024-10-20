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
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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
 *
 */

#include "autoware/costmap_generator/utils/object_map_utils.hpp"

#include <tf2/time.h>

#include <string>
#include <vector>

namespace object_map
{

void FillPolygonAreas(
  grid_map::GridMap & out_grid_map,
  const std::vector<std::vector<geometry_msgs::msg::Point>> & in_points,
  const std::string & in_grid_layer_name, const int in_layer_background_value,
  const int in_fill_color, const int in_layer_min_value, const int in_layer_max_value)
{
  if (!out_grid_map.exists(in_grid_layer_name)) {
    out_grid_map.add(in_grid_layer_name);
  }
  out_grid_map[in_grid_layer_name].setConstant(in_layer_background_value);

  cv::Mat original_image;
  grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
    out_grid_map, in_grid_layer_name, CV_8UC1, in_layer_min_value, in_layer_max_value,
    original_image);

  cv::Mat merged_filled_image = original_image.clone();

  // calculate out_grid_map position
  grid_map::Position map_pos = out_grid_map.getPosition();
  const double origin_x_offset = out_grid_map.getLength().x() / 2.0 - map_pos.x();
  const double origin_y_offset = out_grid_map.getLength().y() / 2.0 - map_pos.y();

  for (const auto & points : in_points) {
    std::vector<cv::Point> cv_polygon;
    for (const auto & p : points) {
      // coordinate conversion for cv image
      const double cv_x =
        (out_grid_map.getLength().y() - origin_y_offset - p.y) / out_grid_map.getResolution();
      const double cv_y =
        (out_grid_map.getLength().x() - origin_x_offset - p.x) / out_grid_map.getResolution();
      cv_polygon.emplace_back(cv_x, cv_y);
    }

    cv::Mat filled_image = original_image.clone();

    std::vector<std::vector<cv::Point>> cv_polygons;
    cv_polygons.push_back(cv_polygon);
    cv::fillPoly(filled_image, cv_polygons, cv::Scalar(in_fill_color));

    merged_filled_image &= filled_image;
  }

  // convert to ROS msg
  grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(
    merged_filled_image, in_grid_layer_name, out_grid_map, in_layer_min_value, in_layer_max_value);
}

}  // namespace object_map
