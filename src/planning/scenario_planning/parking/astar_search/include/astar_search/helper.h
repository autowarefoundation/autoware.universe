/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#pragma once

constexpr double deg2rad(const double deg) { return deg * M_PI / 180.0; }

constexpr double rad2deg(const double rad) { return rad * 180.0 / M_PI; }

constexpr double kmph2mps(const double kmph) { return kmph * 1000.0 / 3600.0; }

constexpr double mps2kmph(const double mps) { return mps * 3600.0 / 1000.0; }

constexpr double normalizeDegree(
  const double deg, const double min_deg = -180, const double max_deg = 180)
{
  const auto value = std::fmod(deg, 360.0);
  if (min_deg < value && value <= max_deg)
    return value;
  else
    return value - std::copysign(360.0, value);
}

constexpr double normalizeRadian(
  const double rad, const double min_rad = -M_PI, const double max_rad = M_PI)
{
  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad < value && value <= max_rad)
    return value;
  else
    return value - std::copysign(2 * M_PI, value);
}
