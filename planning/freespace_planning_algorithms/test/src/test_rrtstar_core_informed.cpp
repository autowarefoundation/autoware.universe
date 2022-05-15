// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "freespace_planning_algorithms/rrtstar_core.hpp"

#include <iostream>

int main()
{
  const rrtstar_core::Pose x_start{0.1, 0.1, 0};
  const rrtstar_core::Pose x_goal{0.1, 0.9, 0.};

  const rrtstar_core::Pose x_lo{0, 0, -6.28};
  const rrtstar_core::Pose x_hi{1., 1., +6.28};
  auto lambda = [](const rrtstar_core::Pose & p) {
    const double score = (p.x - 0.5) * (p.x - 0.5) + (p.y - 0.5) * (p.y - 0.5);
    return score > 0.09;
  };
  const auto cspace = rrtstar_core::CSpace(x_lo, x_hi, 0.2, lambda);
  auto algo = rrtstar_core::RRTStar(x_start, x_goal, 0.3, 0.01, true, cspace);

  clock_t start = clock();
  for (int i = 0; i < 3000; i++) {
    algo.extend();
  }
  clock_t end = clock();
  std::cout << "elapsed time : " << (end - start) / 1000.0 << " [msec]" << std::endl;
  algo.dumpState("/tmp/rrt_result.txt");
}
