// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory_container/trajectory/detail/manipulable_interpolated_array.hpp"

#include <Eigen/Core>
#include <autoware/motion_utils/trajectory_container/interpolator.hpp>

#include <matplotlibcpp17/pyplot.h>

#include <vector>

using autoware::motion_utils::trajectory_container::interpolator::InterpolatorCreator;
using autoware::motion_utils::trajectory_container::interpolator::Stairstep;
using autoware::motion_utils::trajectory_container::trajectory::detail::
  ManipulableInterpolatedArray;

void plot_array(
  const ManipulableInterpolatedArray<double> & arr, matplotlibcpp17::pyplot::PyPlot & plt,
  const std::string & label = "", const std::string & color = "blue")
{
  auto [axis, values] = arr.get_data();
  std::vector<double> axis_vec(axis.begin(), axis.end());
  plt.scatter(Args(axis_vec, values), Kwargs("color"_a = color));

  std::vector<double> x;
  std::vector<double> y;

  for (double i = axis(0); i <= axis(axis.size() - 1); i += 0.01) {
    x.push_back(i);
    y.push_back(arr.compute(i));
  }

  plt.plot(Args(x, y), Kwargs("label"_a = label, "color"_a = color));
}

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  Eigen::VectorXd axis = Eigen::VectorXd::LinSpaced(9, 0.0, 8.0);
  std::vector<double> values(9, 0.0);

  auto interpolator =
    InterpolatorCreator<Stairstep<double>>().set_axis(axis).set_values(values).create();

  ManipulableInterpolatedArray<double> arr(interpolator->clone());

  arr.build(axis, values);

  plot_array(arr, plt, "Original", "red");

  arr(0.5, 1.5) = 1.0;

  arr(2.0, 3.0) = 2.0;

  plot_array(arr, plt, "Modified", "blue");

  plt.legend();
  plt.show();
}
