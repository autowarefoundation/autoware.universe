// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#ifndef COSTMAP_GENERATOR__VISIBILITY_CONTROL_HPP_
#define COSTMAP_GENERATOR__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
#if defined(COSTMAP_GENERATOR_BUILDING_DLL) || defined(COSTMAP_GENERATOR_EXPORTS)
#define COSTMAP_GENERATOR_PUBLIC __declspec(dllexport)
#define COSTMAP_GENERATOR_LOCAL
#else
// defined(COSTMAP_GENERATOR_BUILDING_DLL) || defined(COSTMAP_GENERATOR_EXPORTS)
#define COSTMAP_GENERATOR_PUBLIC __declspec(dllimport)
#define COSTMAP_GENERATOR_LOCAL
#endif
// defined(COSTMAP_GENERATOR_BUILDING_DLL) || defined(COSTMAP_GENERATOR_EXPORTS)
#elif defined(__linux__)
#define COSTMAP_GENERATOR_PUBLIC __attribute__((visibility("default")))
#define COSTMAP_GENERATOR_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define COSTMAP_GENERATOR_PUBLIC __attribute__((visibility("default")))
#define COSTMAP_GENERATOR_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
#error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // COSTMAP_GENERATOR__VISIBILITY_CONTROL_HPP_
