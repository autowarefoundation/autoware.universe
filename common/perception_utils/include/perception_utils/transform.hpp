// Copyright 2022 TIER IV, Inc.
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

#ifndef PERCEPTION_UTILS__TRANSFORM_HPP_
#define PERCEPTION_UTILS__TRANSFORM_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

namespace perception_utils
{

template <class T>
bool transformObjects(
  const T & input_msg, const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  T & output_msg);

}
#endif  // PERCEPTION_UTILS__TRANSFORM_HPP_
