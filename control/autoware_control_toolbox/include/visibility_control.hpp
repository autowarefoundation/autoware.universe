// Copyright 2022 The Autoware Foundation
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

#ifndef AUTOWARE_CONTROL_TOOLBOX__VISIBILITY_CONTROL_HPP
#define AUTOWARE_CONTROL_TOOLBOX__VISIBILITY_CONTROL_HPP

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
#if defined(CDOB_BUILDING_DLL) || defined(CDOB_EXPORTS)
#define CDOB_PUBLIC __declspec(dllexport)
#define CDOB_LOCAL
#else  // defined(CDOB_BUILDING_DLL) || defined(CDOB_EXPORTS)
#define CDOB_PUBLIC __declspec(dllimport)
#define CDOB_LOCAL
#endif  // defined(CDOB_BUILDING_DLL) || defined(CDOB_EXPORTS)
#elif defined(__linux__)
#define ACT_PUBLIC __attribute__((visibility("default")))
#define ACT_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define CDOB_PUBLIC __attribute__((visibility("default")))
#define CDOB_LOCAL __attribute__((visibility("hidden")))
#else
#error "Unsupported Build Configuration"
#endif

#endif //AUTOWARE_CONTROL_TOOLBOX__VISIBILITY_CONTROL_HPP
