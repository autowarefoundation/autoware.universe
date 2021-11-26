// Copyright 2021 the Autoware Foundation
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
/// \file
/// \brief Visibility contorl
#ifndef LGSVL_GROUND_TRUTH__VISIBILITY_CONTROL_HPP_
#define LGSVL_GROUND_TRUTH__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define LGSVL_GROUND_TRUTH_EXPORT __attribute__((dllexport))
#define LGSVL_GROUND_TRUTH_IMPORT __attribute__((dllimport))
#else
#define LGSVL_GROUND_TRUTH_EXPORT __declspec(dllexport)
#define LGSVL_GROUND_TRUTH_IMPORT __declspec(dllimport)
#endif
#ifdef LGSVL_GROUND_TRUTH_BUILDING_LIBRARY
#define LGSVL_GROUND_TRUTH_PUBLIC LGSVL_GROUND_TRUTH_EXPORT
#else
#define LGSVL_GROUND_TRUTH_PUBLIC LGSVL_GROUND_TRUTH_IMPORT
#endif
#define LGSVL_GROUND_TRUTH_PUBLIC_TYPE LGSVL_GROUND_TRUTH_PUBLIC
#define LGSVL_GROUND_TRUTH_LOCAL
#else
#define LGSVL_GROUND_TRUTH_EXPORT __attribute__((visibility("default")))
#define LGSVL_GROUND_TRUTH_IMPORT
#if __GNUC__ >= 4
#define LGSVL_GROUND_TRUTH_PUBLIC __attribute__((visibility("default")))
#define LGSVL_GROUND_TRUTH_LOCAL __attribute__((visibility("hidden")))
#else
#define LGSVL_GROUND_TRUTH_PUBLIC
#define LGSVL_GROUND_TRUTH_LOCAL
#endif
#define LGSVL_GROUND_TRUTH_PUBLIC_TYPE
#endif

#endif  // LGSVL_GROUND_TRUTH__VISIBILITY_CONTROL_HPP_
