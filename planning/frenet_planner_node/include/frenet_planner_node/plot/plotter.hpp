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

#ifndef FRENET_PLANNER_NODE__PLOT__PLOTTER_HPP
#define FRENET_PLANNER_NODE__PLOT__PLOTTER_HPP

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"

#include <qcustomplot.h>
#include <qevent.h>

#include <vector>

namespace frenet_planner_node::plot
{
using sampler_common::FrenetPoint;
using sampler_common::Point;
using sampler_common::Polygon;
using frenet_planner::Trajectory;
class Plotter
{
public:
  Plotter(QCustomPlot * fcplot, QCustomPlot * polyplot);

  inline bool inFrenetRect(const QPoint & p) { return frenet_rect_->rect().contains(p); }
  inline bool inCartesianRect(const QPoint & p) { return cartesian_rect_->rect().contains(p); }
  void plotReferencePath(const std::vector<Point> & reference_path);
  void plotReferencePath(const std::vector<double> & xs, const std::vector<double> & ys);
  void plotTrajectories(const std::vector<Trajectory> & trajectories);
  void plotFrenetTrajectories(const std::vector<Trajectory> & trajectories);
  void plotCartesianTrajectories(const std::vector<Trajectory> & trajectories);
  void plotCommittedTrajectory(const Trajectory & trajectory);
  void plotSelectedTrajectory(const Trajectory & trajectory);
  void plotCurrentPose(const FrenetPoint &, const Point &);
  void plotPolygons(const std::vector<Polygon> & polygons);
  void pixelToCartesian(const QPoint p, double & graph_x, double & graph_y);
  void pixelToFrenet(const QPoint p, double & graph_s, double & graph_d);
  void replot(const bool fc, const bool poly, const bool rescale);

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
  std::vector<QCPCurve *> frenet_trajectories_;
  std::vector<QCPCurve *> cartesian_trajectories_;
  std::vector<QCPCurve *> polygons_;
  QCPCurve * selected_frenet_curve_ = nullptr;
  QCPCurve * selected_cartesian_curve_ = nullptr;
  QCPCurve * committed_frenet_curve_ = nullptr;
  QCPCurve * committed_cartesian_curve_ = nullptr;

  static QCPCurve * trajectoryToFrenetCurve(const Trajectory & trajectory, QCPAxisRect * axis_rect);
  static QCPCurve * trajectoryToCartesianCurve(
    const Trajectory & trajectory, QCPAxisRect * axis_rect);
};

}  // namespace frenet_planner_node::plot

#endif  // FRENET_PLANNER_NODE__PLOT__PLOTTER_HPP
