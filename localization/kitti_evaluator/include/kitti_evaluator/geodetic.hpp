// Copyright 2015-2019 Autoware Foundation
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

#ifndef KITTI_EVALUATOR__GEODETIC_HPP_
#define KITTI_EVALUATOR__GEODETIC_HPP_

#include <vector>
#include <cmath>

namespace geodetic
{
    const double semimajor_axis = 6378137.0;
    const double semiminor_axis = 6356752.31424518;
    const double pi = 3.14159265359;

    std::vector<double> se3_translation(double lat, double lon, double h, double scale) {
        double tx,ty,tz;
        tx = scale * lon * pi * semimajor_axis / 180.0;
        ty = scale * semimajor_axis * log(tan((90.0 + lat) * pi / 360.0));
        tz = h;
        return {tx,ty,tz};
    }
}
#endif
