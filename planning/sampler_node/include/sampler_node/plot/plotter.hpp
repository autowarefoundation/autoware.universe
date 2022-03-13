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

#ifndef SAMPLER_NODE__PLOT__PLOTTER_HPP
#define SAMPLER_NODE__PLOT__PLOTTER_HPP

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"

#include <qcustomplot.h>
#include <qevent.h>

#include <vector>

namespace sampler_node::plot
{
using sampler_common::FrenetPoint;
using sampler_common::Point;
using sampler_common::Polygon;
class Plotter
{
public:
  Plotter(QCustomPlot * fcplot, QCustomPlot * polyplot);

  inline bool inFrenetRect(const QPoint & p) { return frenet_rect_->rect().contains(p); }
  inline bool inCartesianRect(const QPoint & p) { return cartesian_rect_->rect().contains(p); }
  void plotReferencePath(const std::vector<Point> & reference_path);
  void plotReferencePath(const std::vector<double> & xs, const std::vector<double> & ys);
  void plotTrajectories(const std::vector<frenet_planner::Trajectory> & trajectories);
  void plotPaths(const std::vector<sampler_common::Path> & paths);
  void plotFrenetTrajectories(const std::vector<frenet_planner::Trajectory> & trajectories);
  void plotFrenetPaths(const std::vector<frenet_planner::Path> & paths);
  void plotCartesianTrajectories(const std::vector<frenet_planner::Trajectory> & trajectories);
  void plotCartesianPaths(const std::vector<sampler_common::Path> & paths);
  void plotCommittedTrajectory(const frenet_planner::Trajectory & trajectory);
  void plotCommittedPath(const sampler_common::Path & path);
  void plotSelectedTrajectory(const frenet_planner::Trajectory & trajectory);
  void plotSelectedPath(const sampler_common::Path & path);
  void plotCurrentPose(const FrenetPoint &, const Point &);
  void plotPolygons(const std::vector<Polygon> & polygons);
  void pixelToCartesian(const QPoint p, double & graph_x, double & graph_y);
  void pixelToFrenet(const QPoint p, double & graph_s, double & graph_d);
  void replot(const bool fc, const bool poly, const bool rescale);
  void clear();

private:
  QCustomPlot * fcplot_;
  QCustomPlot * polyplot_;
  QCPAxisRect * frenet_rect_;
  QCPAxisRect * cartesian_rect_;
  QCPAxisRect * poly_s_rect_;
  QCPAxisRect * poly_d_rect_;
  QCPGraph * reference_path_;
  QCPGraph * frenet_point_;
  QCPGraph * cartesian_point_;
  std::vector<QCPCurve *> frenet_curves_;
  std::vector<QCPCurve *> cartesian_curves_;
  std::vector<QCPCurve *> polygons_;
  QCPCurve * selected_frenet_curve_ = nullptr;
  QCPCurve * selected_cartesian_curve_ = nullptr;
  QCPCurve * committed_frenet_curve_ = nullptr;
  QCPCurve * committed_cartesian_curve_ = nullptr;

  template <typename T> static QCPCurve * toFrenetCurve(const T & frenet, QCPAxisRect * axis_rect);
  static QCPCurve * toCartesianCurve(const sampler_common::Path & path, QCPAxisRect * axis_rect);
};

}  // namespace sampler_node::plot

#endif  // SAMPLER_NODE__PLOT__PLOTTER_HPP
