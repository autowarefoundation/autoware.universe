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

#ifndef __DATACONTAINERBASE_H
#define __DATACONTAINERBASE_H

#include <ros/ros.h>

namespace velodyne_rawdata
{
class DataContainerBase
{
public:
  virtual void addPoint(
    const float & x, const float & y, const float & z, const uint16_t & ring,
    const uint16_t & azimuth, const float & distance, const float & intensity,
    const double & time_stamp) = 0;
};
}  // namespace velodyne_rawdata
#endif  //__DATACONTAINERBASE_H
