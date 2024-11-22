// Copyright 2024 TIER IV, Inc.
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

#include <autoware/pyplot/pyplot.hpp>

#include <gtest/gtest.h>
#include <pybind11/embed.h>
/*
  very weirdly, you must include <pybind11/stl.h> to pass vector. Although the code
  compiles without it, the executable crashes at runtime
 */
#include <pybind11/stl.h>

#include <vector>

std::vector<double> linspace(const double start, const double end, const size_t N)
{
  if (N < 2) {
    return {};
  }
  if (N == 2) {
    return {start, end};
  }

  std::vector<double> array(N, 0.0);
  double x = start;
  const double diff = (end - start) / (N - 1);
  for (unsigned i = 0; i < N - 1; ++i) {
    array.at(i) = x;
    x += diff;
  }
  array.at(N - 1) = end;
  return array;
}

template <typename F>
std::vector<double> apply(const std::vector<double> & in, F && f)
{
  std::vector<double> out(in.size(), 0.0);
  for (unsigned i = 0; i < in.size(); ++i) {
    out.at(i) = f(in.at(i));
  }
  return out;
}

TEST(PyPlot, single_plot)
{
  py::scoped_interpreter guard{};
  auto plt = autoware::pyplot::import();
  plt.plot(Args(std::vector<int>({1, 3, 2, 4})), Kwargs("color"_a = "blue", "linewidth"_a = 1.0));
  plt.savefig(Args("test_pyplot.png"));
}
