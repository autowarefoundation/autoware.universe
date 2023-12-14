// Copyright 2023 The Autoware Contributors
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

#ifndef LANELET2_MAP_LOADER__LANELET2_DUMMY_PROJECTOR_HPP_
#define LANELET2_MAP_LOADER__LANELET2_DUMMY_PROJECTOR_HPP_

#include <lanelet2_io/Projection.h>

namespace lanelet::projection
{

class DummyProjector : public Projector
{
public:
  BasicPoint3d forward(const GPSPoint &) const override { return {}; }  // NOLINT
  GPSPoint reverse(const BasicPoint3d &) const override { return {}; }
};

}  // namespace lanelet::projection

#endif  // LANELET2_MAP_LOADER__LANELET2_DUMMY_PROJECTOR_HPP_
