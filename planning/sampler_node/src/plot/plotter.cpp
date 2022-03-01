/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
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

#include "sampler_node/plot/plotter.hpp"

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <qcustomplot.h>
#include <qnamespace.h>

namespace sampler_node::plot
{
Plotter::Plotter(QCustomPlot * fcplot, QCustomPlot * polyplot)
: fcplot_(fcplot), polyplot_(polyplot)
{
  // Axis rectangles where we will plot graphs
  frenet_rect_ = new QCPAxisRect(fcplot);
  cartesian_rect_ = new QCPAxisRect(fcplot);
  poly_s_rect_ = new QCPAxisRect(polyplot);
  poly_d_rect_ = new QCPAxisRect(polyplot);

  // Reference path graph
  reference_path_ = fcplot_->addGraph(
    cartesian_rect_->axis(QCPAxis::atBottom), cartesian_rect_->axis(QCPAxis::atLeft));
  reference_path_->setLineStyle(QCPGraph::lsNone);
  reference_path_->setScatterStyle(QCPScatterStyle::ssCircle);
  reference_path_->setPen(QPen(QColor(0, 0, 0)));
  // Plot Layout
  {
    // Frenet/Cartesian graphs with titles : 4 rows
    auto * frenet_title =
      new QCPTextElement(fcplot, "Frenet Trajectories", QFont("sans", 12, QFont::Bold));
    auto * cart_title =
      new QCPTextElement(fcplot, "Cartesian Trajectories", QFont("sans", 12, QFont::Bold));
    fcplot_->plotLayout()->clear();
    fcplot_->plotLayout()->addElement(0, 0, frenet_title);
    fcplot_->plotLayout()->addElement(1, 0, frenet_rect_);
    fcplot_->plotLayout()->addElement(2, 0, cart_title);
    fcplot_->plotLayout()->addElement(3, 0, cartesian_rect_);
  }
  {
    // Frenet polynomial plots with titles : 4 rows
    auto * s_title =
      new QCPTextElement(polyplot_, "Polynomial: s over time", QFont("sans", 12, QFont::Bold));
    auto * d_title =
      new QCPTextElement(polyplot_, "Polynomial: d over time", QFont("sans", 12, QFont::Bold));
    polyplot_->plotLayout()->clear();
    polyplot_->plotLayout()->addElement(0, 0, s_title);
    polyplot_->plotLayout()->addElement(1, 0, poly_s_rect_);
    polyplot_->plotLayout()->addElement(2, 0, d_title);
    polyplot_->plotLayout()->addElement(3, 0, poly_d_rect_);
  }

  // Points for current pose
  frenet_point_ =
    new QCPGraph(frenet_rect_->axis(QCPAxis::atBottom), frenet_rect_->axis(QCPAxis::atLeft));
  cartesian_point_ =
    new QCPGraph(cartesian_rect_->axis(QCPAxis::atBottom), cartesian_rect_->axis(QCPAxis::atLeft));
  frenet_point_->setAdaptiveSampling(false);
  frenet_point_->setLineStyle(QCPGraph::lsNone);
  frenet_point_->setScatterStyle(QCPScatterStyle::ssCircle);
  frenet_point_->setPen(QPen(QBrush(Qt::red), 2));
  cartesian_point_->setAdaptiveSampling(false);
  cartesian_point_->setLineStyle(QCPGraph::lsNone);
  cartesian_point_->setScatterStyle(QCPScatterStyle::ssCircle);
  cartesian_point_->setPen(QPen(QBrush(Qt::red), 2));

  // Enable interactions
  fcplot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  polyplot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

void Plotter::plotReferencePath(const std::vector<Point> & reference_path)
{
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(reference_path.size()));
  ys.reserve(static_cast<int>(reference_path.size()));
  for (const auto & p : reference_path) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  reference_path_->setData(xs, ys);
}

void Plotter::plotReferencePath(const std::vector<double> & xs, const std::vector<double> & ys)
{
  reference_path_->setData(QVector<double>::fromStdVector(xs), QVector<double>::fromStdVector(ys));
}

void Plotter::plotTrajectories(const std::vector<Trajectory> & trajectories)
{
  plotCartesianTrajectories(trajectories);
  plotFrenetTrajectories(trajectories);
}

void Plotter::plotFrenetTrajectories(const std::vector<Trajectory> & trajectories)
{
  for (auto plottable : frenet_trajectories_) fcplot_->removePlottable(plottable);
  polyplot_->clearGraphs();
  frenet_trajectories_.clear();

  for (const auto & trajectory : trajectories) {
    if (trajectory.frenet_points.empty()) {
      continue;
    }
    auto * frenet_curve = trajectoryToFrenetCurve(trajectory, frenet_rect_);
    frenet_curve->setPen(QPen(QColor(0, 0, 255, 100)));
    frenet_trajectories_.push_back(frenet_curve);

    // Plot polynomials
    QVector<double> ss;
    QVector<double> ds;
    QVector<double> ts;
    constexpr double time_resolution = 0.1;
    for (double t = 0; t <= trajectory.duration; t += time_resolution) {
      ts.push_back(t);
      ss.push_back(trajectory.longitudinal_polynomial->position(t));
      ds.push_back(trajectory.lateral_polynomial->position(t));
    }
    auto * poly_s_graph = polyplot_->addGraph(
      poly_s_rect_->axis(QCPAxis::atBottom), poly_s_rect_->axis(QCPAxis::atLeft));
    poly_s_graph->setData(ts, ss);
    auto * poly_d_graph = polyplot_->addGraph(
      poly_d_rect_->axis(QCPAxis::atBottom), poly_d_rect_->axis(QCPAxis::atLeft));
    poly_d_graph->setData(ts, ds);
  }
}

void Plotter::plotCommittedTrajectory(const Trajectory & trajectory)
{
  if (committed_frenet_curve_ != nullptr) fcplot_->removePlottable(committed_frenet_curve_);
  if (committed_cartesian_curve_ != nullptr) fcplot_->removePlottable(committed_cartesian_curve_);
  committed_frenet_curve_ = trajectoryToFrenetCurve(trajectory, frenet_rect_);
  committed_cartesian_curve_ = trajectoryToCartesianCurve(trajectory, cartesian_rect_);
  const auto committed_color = QColor(0, 255, 0);
  committed_frenet_curve_->setPen(QPen(committed_color));
  committed_cartesian_curve_->setPen(QPen(committed_color));
}

void Plotter::plotSelectedTrajectory(const Trajectory & trajectory)
{
  if (selected_frenet_curve_ != nullptr) fcplot_->removePlottable(selected_frenet_curve_);
  if (selected_cartesian_curve_ != nullptr) fcplot_->removePlottable(selected_cartesian_curve_);
  selected_frenet_curve_ = trajectoryToFrenetCurve(trajectory, frenet_rect_);
  selected_cartesian_curve_ = trajectoryToCartesianCurve(trajectory, cartesian_rect_);
  const auto selected_color = QColor(255, 0, 0, 150);
  selected_frenet_curve_->setPen(QPen(selected_color));
  selected_cartesian_curve_->setPen(QPen(selected_color));
}

void Plotter::plotCartesianTrajectories(const std::vector<Trajectory> & trajectories)
{
  static const auto valid_pen = QPen(QColor(0, 0, 255, 100));
  static const auto invalid_pen = QPen(QColor(0, 0, 0, 100));
  for (auto plottable : cartesian_trajectories_) fcplot_->removePlottable(plottable);
  cartesian_trajectories_.clear();

  for (const auto & trajectory : trajectories) {
    if (trajectory.points.empty()) {
      continue;
    }
    auto * cart_curve = trajectoryToCartesianCurve(trajectory, cartesian_rect_);
    cart_curve->setPen(trajectory.valid ? valid_pen : invalid_pen);
    cartesian_trajectories_.push_back(cart_curve);
  }
}

void Plotter::pixelToCartesian(const QPoint p, double & graph_x, double & graph_y)
{
  graph_x = cartesian_rect_->axis(QCPAxis::AxisType::atBottom)->pixelToCoord(p.x());
  graph_y = cartesian_rect_->axis(QCPAxis::AxisType::atLeft)->pixelToCoord(p.y());
}

void Plotter::pixelToFrenet(const QPoint p, double & graph_s, double & graph_d)
{
  graph_s = frenet_rect_->axis(QCPAxis::AxisType::atBottom)->pixelToCoord(p.x());
  graph_d = frenet_rect_->axis(QCPAxis::AxisType::atLeft)->pixelToCoord(p.y());
}

void Plotter::plotCurrentPose(const FrenetPoint & fp, const Point & p)
{
  frenet_point_->setData({fp.s}, {fp.d});
  cartesian_point_->setData({p.x()}, {p.y()});
}

void Plotter::plotPolygons(const std::vector<Polygon> & polygons)
{
  // clear previously drawn polygons
  for (auto plottable : polygons_) fcplot_->removePlottable(plottable);
  polygons_.clear();

  QVector<double> xs;
  QVector<double> ys;
  for (const auto & polygon : polygons) {
    if (polygon.outer().empty()) {
      continue;
    }
    xs.clear();
    ys.clear();
    xs.reserve(static_cast<int>(polygon.outer().size()));
    ys.reserve(static_cast<int>(polygon.outer().size()));
    for (const auto & p : polygon.outer()) {
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
    auto * poly_curve = new QCPCurve(
      cartesian_rect_->axis(QCPAxis::atBottom), cartesian_rect_->axis(QCPAxis::atLeft));
    poly_curve->setPen(Qt::PenStyle::NoPen);
    poly_curve->setBrush(QBrush(QColor(0, 0, 0, 50)));
    poly_curve->setData(xs, ys);
    polygons_.push_back(poly_curve);
  }
}

void Plotter::replot(const bool fc, const bool poly, const bool rescale)
{
  if (fc) {
    if (rescale) {
      fcplot_->rescaleAxes();
    }
    fcplot_->replot();
  } else if (poly) {
    if (rescale) {
      polyplot_->rescaleAxes();
    }
    polyplot_->replot();
  }
}

QCPCurve * Plotter::trajectoryToFrenetCurve(const Trajectory & trajectory, QCPAxisRect * axis_rect)
{
  auto curve = new QCPCurve(axis_rect->axis(QCPAxis::atBottom), axis_rect->axis(QCPAxis::atLeft));
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(trajectory.frenet_points.size()));
  ys.reserve(static_cast<int>(trajectory.frenet_points.size()));
  for (const auto & p : trajectory.frenet_points) {
    xs.push_back(p.s);
    ys.push_back(p.d);
  }
  curve->setData(xs, ys);
  return curve;
}

QCPCurve * Plotter::trajectoryToCartesianCurve(
  const Trajectory & trajectory, QCPAxisRect * axis_rect)
{
  auto curve = new QCPCurve(axis_rect->axis(QCPAxis::atBottom), axis_rect->axis(QCPAxis::atLeft));
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(trajectory.points.size()));
  ys.reserve(static_cast<int>(trajectory.points.size()));
  for (const auto & p : trajectory.points) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  curve->setData(xs, ys);
  return curve;
}

}  // namespace sampler_node::plot
