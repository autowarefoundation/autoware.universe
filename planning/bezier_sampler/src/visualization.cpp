/*
 * Copyright 2021 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <bezier_sampler/visualization.hpp>

namespace motion_planning
{
namespace bezier_sampler
{
void plot(std::vector<Bezier> bs, int nb_points)
{
  for (Bezier b : bs) {
    std::vector<double> xs;
    std::vector<double> ys;
    // plot points of the curve
    for (Eigen::Vector2d point : b.cartesian(nb_points)) {
      xs.push_back(point.x());
      ys.push_back(point.y());
    }
    matplotlibcpp::plot(xs, ys);
  }
  matplotlibcpp::show();
}
void plot(Bezier b, int nb_points)
{
  {
    std::vector<double> xs;
    std::vector<double> ys;
    // plot control points
    for (int i = 0; i < 6; ++i) {
      Eigen::Vector2d point = b.getControlPoints().block<1, 2>(i, 0);
      xs.push_back(point.x());
      ys.push_back(point.y());
    }
    matplotlibcpp::plot(xs, ys, "ro");
  }
  {
    std::vector<double> xs;
    std::vector<double> ys;
    // plot points of the curve
    for (Eigen::Vector2d point : b.cartesian(nb_points)) {
      xs.push_back(point.x());
      ys.push_back(point.y());
    }
    matplotlibcpp::plot(xs, ys);
  }
  matplotlibcpp::xlim(
    b.getControlPoints().col(0).minCoeff() - 1, b.getControlPoints().col(0).maxCoeff() + 1);
  matplotlibcpp::ylim(
    b.getControlPoints().col(1).minCoeff() - 1, b.getControlPoints().col(1).maxCoeff() + 1);
  matplotlibcpp::show();
}
void plot_curvature(Bezier b, int nb_points)
{
  {
    matplotlibcpp::subplot(2, 1, 1);
    std::vector<double> xs;
    std::vector<double> ys;
    // plot points of the curve
    for (Eigen::Vector2d point : b.cartesian(nb_points)) {
      xs.push_back(point.x());
      ys.push_back(point.y());
    }
    matplotlibcpp::plot(xs, ys);
    matplotlibcpp::ylim(
      b.getControlPoints().col(1).minCoeff() - 1, b.getControlPoints().col(1).maxCoeff() + 1);
  }
  {
    matplotlibcpp::subplot(2, 1, 2);
    std::vector<double> xs;
    std::vector<double> ys;
    // plot points of the curve
    for (double t = 0; t <= 1; t += 1.0 / (nb_points - 1)) {
      xs.push_back(t);
      ys.push_back(b.curvature(t));
    }
    matplotlibcpp::plot(xs, ys);
    matplotlibcpp::xlim(0, 1);
    matplotlibcpp::ylim(-0.05, 0.5);
  }
  matplotlibcpp::show();
}
}  // namespace bezier_sampler
}  // namespace motion_planning
