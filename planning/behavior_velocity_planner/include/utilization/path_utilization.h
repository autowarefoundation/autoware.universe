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

#include <autoware_planning_msgs/Path.h>

autoware_planning_msgs::Path interpolatePath(
  const autoware_planning_msgs::Path & path, const double length);
autoware_planning_msgs::Path filterLitterPathPoint(const autoware_planning_msgs::Path & path);
autoware_planning_msgs::Path filterStopPathPoint(const autoware_planning_msgs::Path & path);
