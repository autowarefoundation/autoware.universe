// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_PARTICLE_FILTER__CAMERA_CORRECTOR__FAST_COS_HPP_
#define YABLOC_PARTICLE_FILTER__CAMERA_CORRECTOR__FAST_COS_HPP_

namespace yabloc::modularized_particle_filter
{
struct FastCosSin
{
  FastCosSin(int bin = 90)
  {
    for (int i = 0; i < bin + 1; ++i) {
      cos_.push_back(std::cos(i * M_PI / 180.f));
    }
  }
  float cos(float deg) const
  {
    while (deg < 0) {
      deg += 360;
    }
    while (deg > 360) {
      deg -= 360;
    }
    if (deg < 90) {
      return cos_.at(int(deg));
    } else if (deg < 180) {
      return -cos_.at(int(180 - deg));
    } else if (deg < 270) {
      return -cos_.at(int(deg - 180));
    } else {
      return cos_.at(int(360 - deg));
    }
  }

  float sin(float deg) const { return cos(deg - 90.f); }

private:
  std::vector<float> cos_;
};
}  // namespace yabloc::modularized_particle_filter

#endif  // YABLOC_PARTICLE_FILTER__CAMERA_CORRECTOR__FAST_COS_HPP_