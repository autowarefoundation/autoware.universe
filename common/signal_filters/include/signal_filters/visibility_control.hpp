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
#ifndef SIGNAL_FILTERS__VISIBILITY_CONTROL_HPP_
#define SIGNAL_FILTERS__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIGNAL_FILTERS_EXPORT __attribute__ ((dllexport))
    #define SIGNAL_FILTERS_IMPORT __attribute__ ((dllimport))
  #else
    #define SIGNAL_FILTERS_EXPORT __declspec(dllexport)
    #define SIGNAL_FILTERS_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIGNAL_FILTERS_BUILDING_LIBRARY
    #define SIGNAL_FILTERS_PUBLIC SIGNAL_FILTERS_EXPORT
  #else
    #define SIGNAL_FILTERS_PUBLIC SIGNAL_FILTERS_IMPORT
  #endif
  #define SIGNAL_FILTERS_PUBLIC_TYPE SIGNAL_FILTERS_PUBLIC
  #define SIGNAL_FILTERS_LOCAL
#else
  #define SIGNAL_FILTERS_EXPORT __attribute__ ((visibility("default")))
  #define SIGNAL_FILTERS_IMPORT
  #if __GNUC__ >= 4
    #define SIGNAL_FILTERS_PUBLIC __attribute__ ((visibility("default")))
    #define SIGNAL_FILTERS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIGNAL_FILTERS_PUBLIC
    #define SIGNAL_FILTERS_LOCAL
  #endif
  #define SIGNAL_FILTERS_PUBLIC_TYPE
#endif

#endif  // SIGNAL_FILTERS__VISIBILITY_CONTROL_HPP_
