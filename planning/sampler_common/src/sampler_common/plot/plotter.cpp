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

#include "sampler_common/plot/plotter.hpp"

#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <qcustomplot.h>
#include <qnamespace.h>

namespace sampler_common::plot
{
Plotter::Plotter(QCustomPlot * main_plot) : main_plot_(main_plot)
{
  // Axis rectangles where we will plot graphs
  main_rect_ = new QCPAxisRect(main_plot);

  // Reference path graph
  reference_path_ =
    main_plot_->addGraph(main_rect_->axis(QCPAxis::atBottom), main_rect_->axis(QCPAxis::atLeft));
  reference_path_->setLineStyle(QCPGraph::lsNone);
  reference_path_->setScatterStyle(QCPScatterStyle::ssCircle);
  reference_path_->setPen(QPen(QColor(0, 0, 0)));
  // Plot Layout
  {
    // graphs with title : 2 rows
    auto * title = new QCPTextElement(main_plot, "Cartesian Paths", QFont("sans", 12, QFont::Bold));
    main_plot_->plotLayout()->clear();
    main_plot_->plotLayout()->addElement(2, 0, title);
    main_plot_->plotLayout()->addElement(3, 0, main_rect_);
  }

  // Points for current pose
  current_pose_point_ =
    new QCPGraph(main_rect_->axis(QCPAxis::atBottom), main_rect_->axis(QCPAxis::atLeft));
  current_pose_point_->setAdaptiveSampling(false);
  current_pose_point_->setLineStyle(QCPGraph::lsNone);
  current_pose_point_->setScatterStyle(QCPScatterStyle::ssCircle);
  current_pose_point_->setPen(QPen(QBrush(Qt::red), 2));

  // Enable interactions
  main_plot_->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
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

void Plotter::plotCommittedPath(const Path & path)
{
  if (committed_cartesian_curve_ != nullptr)
    main_plot_->removePlottable(committed_cartesian_curve_);
  committed_cartesian_curve_ = pathToCurve(path, main_rect_);
  const auto committed_color = QColor(0, 255, 0);
  committed_cartesian_curve_->setPen(QPen(committed_color));
}

void Plotter::plotSelectedPath(const Path & path)
{
  if (selected_cartesian_curve_ != nullptr) main_plot_->removePlottable(selected_cartesian_curve_);
  selected_cartesian_curve_ = pathToCurve(path, main_rect_);
  const auto selected_color = QColor(255, 0, 0, 150);
  selected_cartesian_curve_->setPen(QPen(selected_color));
}

void Plotter::plotPaths(const std::vector<Path> & paths)
{
  static const auto valid_pen = QPen(QColor(0, 0, 255, 100));
  static const auto invalid_pen = QPen(QColor(0, 0, 0, 100));
  for (auto plottable : paths_) main_plot_->removePlottable(plottable);
  paths_.clear();

  for (const auto & path : paths) {
    if (path.points.empty()) {
      continue;
    }
    auto * cart_curve = pathToCurve(path, main_rect_);
    cart_curve->setPen(path.valid ? valid_pen : invalid_pen);
    paths_.push_back(cart_curve);
  }
}

void Plotter::plotCurrentPose(const Point & p) { current_pose_point_->setData({p.x()}, {p.y()}); }

void Plotter::plotPolygons(const std::vector<Polygon> & polygons)
{
  // clear previously drawn polygons
  for (auto plottable : polygons_) main_plot_->removePlottable(plottable);
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
    auto * poly_curve =
      new QCPCurve(main_rect_->axis(QCPAxis::atBottom), main_rect_->axis(QCPAxis::atLeft));
    poly_curve->setPen(Qt::PenStyle::NoPen);
    poly_curve->setBrush(QBrush(QColor(0, 0, 0, 50)));
    poly_curve->setData(xs, ys);
    polygons_.push_back(poly_curve);
  }
}

void Plotter::replot(const bool rescale)
{
  if (rescale) {
    main_plot_->rescaleAxes();
  }
  main_plot_->replot();
}

QCPCurve * Plotter::pathToCurve(const Path & path, QCPAxisRect * axis_rect)
{
  auto curve = new QCPCurve(axis_rect->axis(QCPAxis::atBottom), axis_rect->axis(QCPAxis::atLeft));
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(path.points.size()));
  ys.reserve(static_cast<int>(path.points.size()));
  for (const auto & p : path.points) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  curve->setData(xs, ys);
  return curve;
}

}  // namespace sampler_common::plot
