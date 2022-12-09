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

#include "sampler_node/gui/window.hpp"

#include "frenet_planner/frenet_planner.hpp"
#include "qcolor.h"
#include "qtablewidget.h"
#include "qvariant.h"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/gui/ui.hpp"

#include <qcustomplot.h>
#include <qlineedit.h>
#include <qnamespace.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qtabwidget.h>
#include <qtextedit.h>

#include <cmath>
#include <iostream>

namespace sampler_node::gui
{
MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui_(new Ui::MainWindow)
{
  const auto ref_path_color = QColor(150, 0, 0);
  const auto raw_path_color = QColor(150, 150, 255);
  const auto candidate_color = QColor(0, 0, 150);
  // const auto prev_path_color = QColor(0, 150, 0);
  ui_->setupUi(this);
  // Output tab
  ui_->output_pos->addGraph();
  ui_->output_pos->graph()->setLineStyle(QCPGraph::lsNone);
  ui_->output_pos->graph()->setScatterStyle(QCPScatterStyle::ssCircle);
  ui_->output_pos->xAxis->setScaleRatio(ui_->output_pos->yAxis);
  ui_->output_pos->xAxis->setLabel("x position (m)");
  ui_->output_pos->yAxis->setLabel("y position (m)");
  output_pos_curve_ = new QCPCurve(ui_->output_pos->xAxis, ui_->output_pos->yAxis);
  output_pos_curve_->setPen(QPen(candidate_color));
  output_pos_raw_path_curve_ = new QCPCurve(ui_->output_pos->xAxis, ui_->output_pos->yAxis);
  output_pos_raw_path_curve_->setPen(QPen(raw_path_color));
  output_pos_arrow_ = new QCPItemLine(ui_->output_pos);
  output_pos_arrow_->setHead(QCPLineEnding::esSpikeArrow);
  output_pos_arrow_->setPen(QPen(candidate_color));
  // First graph for the output graph, Second graph for the current value point
  ui_->output_vel->addGraph();
  ui_->output_vel->addGraph();
  ui_->output_vel->graph()->setLineStyle(QCPGraph::lsNone);
  ui_->output_vel->graph()->setScatterStyle(QCPScatterStyle::ssCircle);
  ui_->output_vel->xAxis->setLabel("time (s)");
  ui_->output_vel->yAxis->setLabel("velocity (m/s)");
  ui_->output_acc->addGraph();
  ui_->output_acc->addGraph();
  ui_->output_acc->graph()->setLineStyle(QCPGraph::lsNone);
  ui_->output_acc->graph()->setScatterStyle(QCPScatterStyle::ssCircle);
  ui_->output_acc->xAxis->setLabel("time (s)");
  ui_->output_acc->yAxis->setLabel("acceleration (m/s²)");
  ui_->output_jerk->addGraph();
  ui_->output_jerk->addGraph();
  ui_->output_jerk->xAxis->setLabel("time (s)");
  ui_->output_jerk->yAxis->setLabel("jerk (m/s³)");
  // Set interactions
  ui_->output_pos->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui_->output_vel->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui_->output_acc->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
  ui_->output_jerk->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

  // Inputs tab
  ui_->input_path->addGraph();
  ui_->input_path->addGraph();
  ui_->input_path->xAxis->setScaleRatio(ui_->output_pos->yAxis);
  ui_->input_path->xAxis->setLabel("x position (m)");
  ui_->input_path->yAxis->setLabel("y position (m)");
  input_path_raw_curve_ = new QCPCurve(ui_->input_path->xAxis, ui_->input_path->yAxis);
  input_path_raw_curve_->setPen(QPen(raw_path_color));
  input_path_smooth_curve_ = new QCPCurve(ui_->input_path->xAxis, ui_->input_path->yAxis);
  input_path_smooth_curve_->setPen(QPen(ref_path_color));
  input_path_smooth_curve_->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross));
  input_path_arrow_ = new QCPItemLine(ui_->output_pos);
  input_path_arrow_->setHead(QCPLineEnding::esSpikeArrow);
  ui_->input_curvature->addGraph();
  ui_->input_curvature->addGraph();
  ui_->input_curvature->xAxis->setLabel("longitudinal position (m)");
  ui_->input_curvature->yAxis->setLabel("curvature (m⁻¹)");
  ui_->input_velocity->addGraph();
  ui_->input_velocity->xAxis->setLabel("longitudinal position (m)");
  ui_->input_velocity->yAxis->setLabel("velocity (m/s)");

  // Candidates tab
  ui_->candidates_table->setColumnCount(3);
  ui_->candidates_table->setHorizontalHeaderLabels({"id", "valid", "cost"});
  ui_->cand_pos->addGraph();
  ui_->cand_pos->xAxis->setScaleRatio(ui_->cand_pos->yAxis);
  cand_pos_curve_ = new QCPCurve(ui_->cand_pos->xAxis, ui_->cand_pos->yAxis);
  cand_path_curve_ = new QCPCurve(ui_->cand_pos->xAxis, ui_->cand_pos->yAxis);
  cand_path_curve_->setPen(QPen(ref_path_color));
  ui_->cand_vel->addGraph();
  ui_->cand_acc->addGraph();
  ui_->cand_jerk->addGraph();
  connect(ui_->candidates_table, &QTableWidget::cellClicked, [&](int row, int col) {
    (void)col;
    const auto id = ui_->candidates_table->item(row, 0)->text().toInt();
    plotCandidate(candidates_[id]);
  });

  // Pruning tab
  pruning_nb_violations_bars_ =
    new QCPBars(ui_->pruning_tab_plot->xAxis, ui_->pruning_tab_plot->yAxis);
  pruning_nb_violations_bars_->setName("Hard constraints violations");
  // prepare x axis with country labels:
  const QVector<QString> labels(
    {"Invalid", "Collision", "Drivable Area", "Velocity", "Acceleration", "Curvature", "Yaw Rate"});
  QVector<double> ticks;
  for (double i = 0.0; i < labels.size(); ++i) ticks << i;
  QSharedPointer<QCPAxisTickerText> ticker(new QCPAxisTickerText);
  ticker->addTicks(ticks, labels);
  ui_->pruning_tab_plot->xAxis->setTicker(ticker);
  ui_->pruning_tab_plot->xAxis->setTickLabelRotation(60);
  ui_->pruning_tab_plot->xAxis->setSubTicks(false);
  ui_->pruning_tab_plot->xAxis->grid()->setVisible(false);
  ui_->pruning_tab_plot->xAxis->setRange(-1.0, static_cast<double>(labels.size()));
  ui_->pruning_tab_plot->yAxis->setLabel("Number of violations");
  pruning_nb_violations_max_line_ = new QCPItemStraightLine(ui_->pruning_tab_plot);
}

MainWindow::~MainWindow() { delete ui_; }

void MainWindow::setStatus(
  const size_t nb_trajs, const double compute_time_ms, const double plot_time_ms)
{
  ui_->statusbar->showMessage(
    QString("Generated Trajectories: %1 | Compute time: %2ms | Render time %3ms")
      .arg(
        QString::number(nb_trajs), QString::number(compute_time_ms),
        QString::number(plot_time_ms)));
}

void MainWindow::update()
{
  QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
  replot();
}

void MainWindow::replot()
{
  /*
  if (ui_->tabWidget->currentWidget() == ui_->frenet_cartesian_plot) {
    plotter_->replot(true, false, ui_->autoscale_toggle_button->isChecked());
  } else if (ui_->tabWidget->currentWidget() == ui_->frenet_polynomials_plot) {
    plotter_->replot(false, true, ui_->autoscale_toggle_button->isChecked());
  }
  */
}

void MainWindow::plotSelected(const sampler_common::Trajectory & trajectory)
{
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(trajectory.points.size()));
  ys.reserve(static_cast<int>(trajectory.points.size()));
  for (const auto & p : trajectory.points) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  output_pos_curve_->setData(xs, ys);
  ui_->output_vel->graph(0)->setData(
    QVector<double>::fromStdVector(trajectory.times),
    QVector<double>::fromStdVector(trajectory.longitudinal_velocities));
  ui_->output_acc->graph(0)->setData(
    QVector<double>::fromStdVector(trajectory.times),
    QVector<double>::fromStdVector(trajectory.longitudinal_accelerations));
  // TODO(Maxime CLEMENT): uncomment once jerk is calculated
  // ui_->output_jerk->graph(0)->setData(QVector<double>::fromStdVector(trajectory.times),
  // QVector<double>::fromStdVector(trajectory.jerks));
  ui_->output_pos->replot();
  ui_->output_pos->rescaleAxes();
  ui_->output_vel->replot();
  ui_->output_vel->rescaleAxes();
  ui_->output_acc->replot();
  ui_->output_acc->rescaleAxes();
  ui_->output_jerk->replot();
  ui_->output_jerk->rescaleAxes();
}

void MainWindow::plotInputs(
  const autoware_auto_planning_msgs::msg::Path & raw_path,
  const sampler_common::transform::Spline2D & spline_path,
  const sampler_common::Configuration & current_configuration)
{
  QVector<double> xs;
  QVector<double> ys;
  // Raw path
  if (!raw_path.points.empty()) {
    xs.reserve(static_cast<int>(raw_path.points.size()));
    ys.reserve(static_cast<int>(raw_path.points.size()));
    for (const auto & p : raw_path.points) {
      xs.push_back(p.pose.position.x);
      ys.push_back(p.pose.position.y);
    }
    xs.clear();
    ys.clear();
    output_pos_raw_path_curve_->setData(xs, ys);
    input_path_raw_curve_->setData(xs, ys);
    auto prev_point = raw_path.points.front();
    auto s = 0.0;
    for (auto p : raw_path.points) {
      const auto dx = prev_point.pose.position.x - p.pose.position.x;
      const auto dy = prev_point.pose.position.y - p.pose.position.y;
      s += std::hypot(dx, dy);
      xs.push_back(s);
      ys.push_back(p.longitudinal_velocity_mps);
      prev_point = p;
    }
    ui_->input_velocity->graph()->setData(xs, ys);
  }
  // Smooth path
  xs.clear();
  ys.clear();
  for (auto s = spline_path.firstS(); s <= spline_path.lastS(); s += 0.1) {
    const auto p = spline_path.cartesian(s);
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  input_path_smooth_curve_->setData(xs, ys);
  cand_path_curve_->setData(xs, ys);
  xs.clear();
  ys.clear();
  constexpr auto resolution = 0.1;
  for (auto s = spline_path.firstS(); s <= spline_path.lastS(); s += resolution) {
    xs.push_back(s);
    ys.push_back(spline_path.curvature(s));
  }
  ui_->input_curvature->graph(0)->setData(xs, ys);
  // Arrow
  const auto end_x = std::cos(current_configuration.heading) * current_configuration.velocity +
                     current_configuration.pose.x();
  const auto end_y = std::sin(current_configuration.heading) * current_configuration.velocity +
                     current_configuration.pose.y();
  input_path_arrow_->start->setCoords(
    current_configuration.pose.x(), current_configuration.pose.y());
  input_path_arrow_->end->setCoords(end_x, end_y);
  output_pos_arrow_->start->setCoords(
    current_configuration.pose.x(), current_configuration.pose.y());
  output_pos_arrow_->end->setCoords(end_x, end_y);
  // Current config
  ui_->output_pos->graph()->setData(
    {current_configuration.pose.x()}, {current_configuration.pose.y()});
  ui_->output_vel->graph()->setData({0.0}, {current_configuration.velocity});
  ui_->output_acc->graph()->setData({0.0}, {current_configuration.acceleration});
  ui_->input_curvature->graph()->setData({0.0}, {current_configuration.curvature});
  ui_->input_path->rescaleAxes();
  ui_->input_path->replot();
  ui_->output_pos->rescaleAxes();
  ui_->output_pos->replot();
  ui_->input_curvature->rescaleAxes();
  ui_->input_curvature->replot();
  ui_->input_velocity->rescaleAxes();
  ui_->input_velocity->replot();
}

void MainWindow::fillCandidatesTable(const std::vector<sampler_common::Trajectory> & candidates)
{
  candidates_ = candidates;
  auto * table = ui_->candidates_table;
  table->setSortingEnabled(false);
  table->clearContents();
  table->setRowCount(candidates.size());
  for (auto i = 0lu; i < candidates.size(); ++i) {
    const auto candidate = candidates[i];
    table->setItem(i, 0, new QTableWidgetItem(tr("%1").arg(i)));
    table->setItem(
      i, 1, new QTableWidgetItem(tr("%1").arg(candidate.constraint_results.isValid())));
    auto * item = new QTableWidgetItem();
    item->setData(Qt::ItemDataRole::DisplayRole, qVariantFromValue(candidate.cost));
    table->setItem(i, 2, item);
  }
  table->setSortingEnabled(true);
}

void MainWindow::plotNbViolatedConstraints(
  const std::vector<sampler_common::Trajectory> & candidates)
{
  const QVector<double> indexes({0, 1, 2, 3, 4, 5, 6});
  QVector<double> nb_violations(indexes.size(), 0.0);
  for (const auto & traj : candidates) {
    if (!traj.constraint_results.isValid()) ++nb_violations[0];
    if (!traj.constraint_results.collision) ++nb_violations[1];
    if (!traj.constraint_results.drivable_area) ++nb_violations[2];
    if (!traj.constraint_results.velocity) ++nb_violations[3];
    if (!traj.constraint_results.acceleration) ++nb_violations[4];
    if (!traj.constraint_results.curvature) ++nb_violations[5];
    if (!traj.constraint_results.yaw_rate) ++nb_violations[6];
  }
  pruning_nb_violations_bars_->setData(indexes, nb_violations);
  pruning_nb_violations_max_line_->point1->setCoords(0, candidates.size());
  pruning_nb_violations_max_line_->point2->setCoords(nb_violations.size(), candidates.size());
  ui_->pruning_tab_plot->yAxis->setRange(0.0, candidates.size() + candidates.size() * 0.05);
  ui_->pruning_tab_plot->replot();
}

void MainWindow::plotCandidate(const sampler_common::Trajectory & trajectory)
{
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(trajectory.points.size()));
  ys.reserve(static_cast<int>(trajectory.points.size()));
  for (const auto & p : trajectory.points) {
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  cand_pos_curve_->setData(xs, ys);
  ui_->cand_vel->graph(0)->setData(
    QVector<double>::fromStdVector(trajectory.times),
    QVector<double>::fromStdVector(trajectory.longitudinal_velocities));
  ui_->cand_acc->graph(0)->setData(
    QVector<double>::fromStdVector(trajectory.times),
    QVector<double>::fromStdVector(trajectory.longitudinal_accelerations));
  ui_->cand_jerk->graph(0)->setData(
    QVector<double>::fromStdVector(trajectory.lengths),
    QVector<double>::fromStdVector(trajectory.curvatures));

  ui_->cand_pos->rescaleAxes();
  ui_->cand_pos->replot();
  ui_->cand_vel->rescaleAxes();
  ui_->cand_vel->replot();
  ui_->cand_acc->rescaleAxes();
  ui_->cand_acc->replot();
  ui_->cand_jerk->rescaleAxes();
  ui_->cand_jerk->replot();
}

void MainWindow::plotObstacles(const sampler_common::Constraints & constraints)
{
  for (auto plottable : obstacle_polygons_) ui_->obstacles_tab_plot->removePlottable(plottable);
  obstacle_polygons_.clear();
  QVector<double> xs;
  QVector<double> ys;
  auto * graph = ui_->obstacles_tab_plot;
  for (const auto & obs : constraints.dynamic_obstacles) {
    for (const auto & poly : obs.footprint_per_time) {
      xs.clear();
      ys.clear();
      xs.reserve(static_cast<int>(poly.outer().size()));
      ys.reserve(static_cast<int>(poly.outer().size()));
      for (const auto & p : poly.outer()) {
        xs.push_back(p.x());
        ys.push_back(p.y());
      }
      auto * curve = new QCPCurve(graph->xAxis, graph->yAxis);
      curve->setPen(Qt::PenStyle::NoPen);
      curve->setBrush(QBrush(QColor(0, 0, 0, 50)));
      curve->setData(xs, ys);
      obstacle_polygons_.push_back(curve);
    }
  }
  ui_->obstacles_tab_plot->rescaleAxes();
  ui_->obstacles_tab_plot->replot();
}

}  // namespace sampler_node::gui
