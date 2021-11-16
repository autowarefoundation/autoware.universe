// Copyright 2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#ifndef REFERENCE_TRACKING_CONTROLLER__VISIBILITY_CONTROL_HPP_
#define REFERENCE_TRACKING_CONTROLLER__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define REFERENCE_TRACKING_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define REFERENCE_TRACKING_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define REFERENCE_TRACKING_CONTROLLER_EXPORT __declspec(dllexport)
    #define REFERENCE_TRACKING_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef REFERENCE_TRACKING_CONTROLLER_BUILDING_LIBRARY
    #define REFERENCE_TRACKING_CONTROLLER_PUBLIC REFERENCE_TRACKING_CONTROLLER_EXPORT
  #else
    #define REFERENCE_TRACKING_CONTROLLER_PUBLIC REFERENCE_TRACKING_CONTROLLER_IMPORT
  #endif
  #define REFERENCE_TRACKING_CONTROLLER_PUBLIC_TYPE REFERENCE_TRACKING_CONTROLLER_PUBLIC
  #define REFERENCE_TRACKING_CONTROLLER_LOCAL
#else
  #define REFERENCE_TRACKING_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define REFERENCE_TRACKING_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define REFERENCE_TRACKING_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define REFERENCE_TRACKING_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define REFERENCE_TRACKING_CONTROLLER_PUBLIC
    #define REFERENCE_TRACKING_CONTROLLER_LOCAL
  #endif
  #define REFERENCE_TRACKING_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // REFERENCE_TRACKING_CONTROLLER__VISIBILITY_CONTROL_HPP_
