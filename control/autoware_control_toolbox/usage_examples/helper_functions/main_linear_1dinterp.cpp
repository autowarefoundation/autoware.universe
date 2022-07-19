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

#include "utils_act/writetopath.hpp"
#include "utils_act/act_utils.hpp"
#include <vector>
#include <algorithm>

int main()
{
  auto log_path = getOutputPath() / "interp_1d";

  // Generate a  sinusoidal signal with arc-length parametrization. This is our test signal.
  size_t const num_of_points = 100; // number of points in the signal.

  // Generate x.
  std::vector<double> xvec = ns_utils::linspace(0.0, 13.0, num_of_points + 1);

  // Generate y = sin(x).
  std::vector<double> yvec;

  std::transform(xvec.cbegin(), xvec.cend(), std::back_inserter(yvec), [](auto const &x)
  {
    return sin(x);
  });

  writeToFile(log_path, xvec, "xvec");
  writeToFile(log_path, yvec, "yvec");

  /**
   * Piecewise interpolation
   * */
  size_t const nnew = 60; // number of points in the signal.
  std::vector<double> xnew = ns_utils::linspace(-2.0, 18.0, nnew + 1);

  std::vector<double> ypw;

  for (auto const &x : xnew)
  {
    double ynew{};
    auto is_interpolated = ns_utils::interp1d_linear(xvec, yvec, x, ynew);

    ns_utils::print("is interpolated ", is_interpolated);
    ypw.emplace_back(ynew);
  }

  writeToFile(log_path, xnew, "xnew");
  writeToFile(log_path, ypw, "ypw");

  /**
   * Vectorwise
   * */
  std::vector<double> yvw;
  ns_utils::interp1d_linear(xvec, yvec, xnew, yvw);
  writeToFile(log_path, yvw, "yvw");

}