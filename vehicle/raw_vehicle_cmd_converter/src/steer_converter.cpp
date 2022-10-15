//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "raw_vehicle_cmd_converter/steer_converter.hpp"

#include "interpolation/linear_interpolation.hpp"

#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{

bool SteerMap::readSteerMapFromCSV(const std::string & csv_path)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    return false;
  }

  vehicle_name_ = table[0][0];
  steer_index_ = CSVLoader::getRowIndex(table);
  output_index_ = CSVLoader::getColumnIndex(table);
  steer_map_ = CSVLoader::getMap(table);
  return true;
}

void SteerMap::getSteer(double steer_rate, double steer, double & output) const
{
  steer = CSVLoader::clampValue(steer, steer_index_, "steer: steer");

  std::vector<double> steer_rate_interp = {};
  for (std::vector<double> steer_angle_velocities : steer_map_) {
    steer_rate_interp.push_back(interpolation::lerp(steer_index_, steer_angle_velocities, steer));
  }
  if (steer_rate < steer_rate_interp.front()) {
    steer_rate = steer_rate_interp.front();
  } else if (steer_rate_interp.back() < steer_rate) {
    steer_rate = steer_rate_interp.back();
  }
  output = interpolation::lerp(steer_rate_interp, output_index_, steer_rate);
}
}  // namespace raw_vehicle_cmd_converter
