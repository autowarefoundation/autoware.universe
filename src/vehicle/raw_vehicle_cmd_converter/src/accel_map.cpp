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
 */

#include "raw_vehicle_cmd_converter/accel_map.h"

AccelMap::AccelMap() {}

AccelMap::~AccelMap() {}

bool AccelMap::readAccelMapFromCSV(std::string csv_path)
{
  CSVLoader csv(csv_path);
  std::vector<std::vector<std::string>> table;

  if (!csv.readCSV(table)) {
    ROS_ERROR("[Accel Map] Cannot open %s", csv_path.c_str());
    return false;
  }

  if (table[0].size() < 2) {
    ROS_ERROR(
      "[Accel Map] Cannot read %s. CSV file should have at least 2 column", csv_path.c_str());
    return false;
  }
  vehicle_name_ = table[0][0];
  for (unsigned int i = 1; i < table[0].size(); i++) {
    vel_index_.push_back(std::stod(table[0][i]));
  }

  for (unsigned int i = 1; i < table.size(); i++) {
    if (table[0].size() != table[i].size()) {
      ROS_ERROR(
        "[Accel Map] Cannot read %s. Each row should have a same number of columns",
        csv_path.c_str());
      return false;
    }
    throttle_index_.push_back(std::stod(table[i][0]));
    std::vector<double> accs;
    for (unsigned int j = 1; j < table[i].size(); j++) {
      accs.push_back(std::stod(table[i][j]));
    }
    accel_map_.push_back(accs);
  }

  return true;
}

bool AccelMap::getThrottle(double acc, double vel, double & throttle)
{
  LinearInterpolate linear_interp;
  std::vector<double> accs_interpolated;

  if (vel < vel_index_.front()) {
    ROS_WARN_DELAYED_THROTTLE(
      1.0,
      "[Accel Map] Exceeding the vel range. Current vel: %f < min vel on map: %f. Use min "
      "velocity.",
      vel, vel_index_.front());
    vel = vel_index_.front();
  } else if (vel_index_.back() < vel) {
    ROS_WARN_DELAYED_THROTTLE(
      1.0,
      "[Accel Map] Exceeding the vel range. Current vel: %f > max vel on map: %f. Use max "
      "velocity.",
      vel, vel_index_.back());
    vel = vel_index_.back();
  }

  // (throttle, vel, acc) map => (throttle, acc) map by fixing vel
  for (std::vector<double> accs : accel_map_) {
    double acc_interpolated;
    linear_interp.interpolate(vel_index_, accs, vel, acc_interpolated);
    accs_interpolated.push_back(acc_interpolated);
  }

  // calculate throttle
  // When the desired acceleration is smaller than the throttle area, return false => brake sequence
  // When the desired acceleration is greater than the throttle area, return max throttle
  if (acc < accs_interpolated.front()) {
    return false;
  } else if (accs_interpolated.back() < acc) {
    throttle = throttle_index_.back();
    return true;
  }
  linear_interp.interpolate(accs_interpolated, throttle_index_, acc, throttle);

  return true;
}
