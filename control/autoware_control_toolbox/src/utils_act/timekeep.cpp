// Copyright 2022 The Autoware Foundation.
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

#include "utils_act/timekeep.hpp"

double tic()
{
  struct timespec t{};
  clock_gettime(CLOCK_REALTIME, &t);
  return double(t.tv_sec) * 1000. + double(t.tv_nsec) / 1000000.;
}

double toc(double start)
{
  return tic() - start;
}
