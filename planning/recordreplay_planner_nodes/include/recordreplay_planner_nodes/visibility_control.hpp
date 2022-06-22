// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho
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

#ifndef RECORDREPLAY_PLANNER_NODES__VISIBILITY_CONTROL_HPP_
#define RECORDREPLAY_PLANNER_NODES__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
  #if defined(RECORDREPLAY_PLANNER_NODES_BUILDING_DLL) || \
  defined(RECORDREPLAY_PLANNER_NODES_EXPORTS)
    #define RECORDREPLAY_PLANNER_NODES_PUBLIC __declspec(dllexport)
    #define RECORDREPLAY_PLANNER_NODES_LOCAL
  #else
// defined(RECORDREPLAY_PLANNER_NODES_BUILDING_DLL) || defined(RECORDREPLAY_PLANNER_NODES_EXPORTS)
    #define RECORDREPLAY_PLANNER_NODES_PUBLIC __declspec(dllimport)
    #define RECORDREPLAY_PLANNER_NODES_LOCAL
  #endif
// defined(RECORDREPLAY_PLANNER_NODES_BUILDING_DLL) || defined(RECORDREPLAY_PLANNER_NODES_EXPORTS)
#elif defined(__linux__)
  #define RECORDREPLAY_PLANNER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RECORDREPLAY_PLANNER_NODES_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define RECORDREPLAY_PLANNER_NODES_PUBLIC __attribute__((visibility("default")))
  #define RECORDREPLAY_PLANNER_NODES_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
  #error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)

#endif  // RECORDREPLAY_PLANNER_NODES__VISIBILITY_CONTROL_HPP_
