// Copyright 2022 Tier IV, Inc.
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

#ifndef SAMPLER_COMMON__PLOT__PLOTTER_HPP
#define SAMPLER_COMMON__PLOT__PLOTTER_HPP

#include "sampler_common/structures.hpp"

#include <qcustomplot.h>
#include <qevent.h>

#include <vector>

namespace sampler_common::plot
{
class Plotter
{
public:
  explicit Plotter(QCustomPlot * main_plot);

  void plotReferencePath(const std::vector<Point> & reference_path);
  void plotReferencePath(const std::vector<double> & xs, const std::vector<double> & ys);
  void plotPaths(const std::vector<Path> & paths);
  void plotCommittedPath(const Path & path);
  void plotSelectedPath(const Path & path);
  void plotCurrentPose(const Point &);
  void plotPolygons(const std::vector<Polygon> & polygons);
  void replot(const bool rescale);

private:
  QCustomPlot * main_plot_;
  QCPAxisRect * main_rect_;
  QCPGraph * reference_path_;
  QCPGraph * current_pose_point_;
  std::vector<QCPCurve *> paths_;
  std::vector<QCPCurve *> polygons_;
  QCPCurve * selected_cartesian_curve_ = nullptr;
  QCPCurve * committed_frenet_curve_ = nullptr;
  QCPCurve * committed_cartesian_curve_ = nullptr;

  static QCPCurve * pathToCurve(const Path & path, QCPAxisRect * axis_rect);
};

}  // namespace sampler_common::plot

#endif
