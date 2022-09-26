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
  ui_->setupUi(this);
  // Output tab
  ui_->output_pos->addGraph();
  ui_->output_pos->graph()->setLineStyle(QCPGraph::lsNone);
  ui_->output_pos->graph()->setScatterStyle(QCPScatterStyle::ssCircle);
  ui_->output_pos->xAxis->setScaleRatio(ui_->output_pos->yAxis);
  ui_->output_pos->xAxis->setLabel("x position (m)");
  ui_->output_pos->yAxis->setLabel("y position (m)");
  output_pos_curve_ = new QCPCurve(ui_->output_pos->xAxis, ui_->output_pos->yAxis);
  output_pos_curve_->setPen(QPen(QColor(150, 0, 0)));
  output_pos_raw_path_curve_ = new QCPCurve(ui_->output_pos->xAxis, ui_->output_pos->yAxis);
  output_pos_raw_path_curve_->setPen(QPen(QColor(0, 0, 0)));
  output_pos_arrow_ = new QCPItemLine(ui_->output_pos);
  output_pos_arrow_->setHead(QCPLineEnding::esSpikeArrow);
  output_pos_arrow_->setPen(QPen(QColor(0, 0, 150)));
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

  // Inputs tab
  ui_->input_path->addGraph();
  ui_->input_path->addGraph();
  ui_->input_path->xAxis->setScaleRatio(ui_->output_pos->yAxis);
  ui_->input_path->xAxis->setLabel("x position (m)");
  ui_->input_path->yAxis->setLabel("y position (m)");
  input_path_raw_curve_ = new QCPCurve(ui_->input_path->xAxis, ui_->input_path->yAxis);
  input_path_smooth_curve_ = new QCPCurve(ui_->input_path->xAxis, ui_->input_path->yAxis);
  input_path_arrow_ = new QCPItemLine(ui_->output_pos);
  input_path_arrow_->setHead(QCPLineEnding::esSpikeArrow);
  ui_->input_curvature->addGraph();
  ui_->input_curvature->addGraph();
  ui_->input_curvature->xAxis->setLabel("longitudinal position (m)");
  ui_->input_curvature->yAxis->setLabel("curvature (m⁻¹)");

  // Candidates tab
  ui_->candidates_table->setColumnCount(3);
  ui_->candidates_table->setHorizontalHeaderLabels({"id", "valid", "cost"});
  ui_->cand_pos->addGraph();
  ui_->cand_pos->xAxis->setScaleRatio(ui_->cand_pos->yAxis);
  cand_pos_curve_ = new QCPCurve(ui_->cand_pos->xAxis, ui_->cand_pos->yAxis);
  ui_->cand_vel->addGraph();
  ui_->cand_acc->addGraph();
  ui_->cand_jerk->addGraph();
  connect(ui_->candidates_table, &QTableWidget::cellClicked, [&](int row, int col) {
    (void)col;
    const auto id = ui_->candidates_table->item(row, 0)->text().toInt();
    plotCandidate(candidates_[id]);
  });

  /*
  const auto & fcplot = ui_->frenet_cartesian_plot;
  const auto & polyplot = ui_->frenet_polynomials_plot;
  plotter_ = std::make_shared<Plotter>(fcplot, polyplot);

  // Signals
  /// Replot when tab is changed
  connect(ui_->tabWidget, &QTabWidget::currentChanged, [&](const auto) { replot(); });
  /// Generate trajectories when button is clicked
  connect(ui_->autoscale_toggle_button, &QPushButton::clicked, [&]() { replot(); });
  /// Regenerate trajectories whenever the inputs are changed
  auto input_list = ui_->initial_states_box->findChildren<QDoubleSpinBox *>();
  input_list.append(ui_->target_states_box->findChildren<QDoubleSpinBox *>());
  // for (const auto * doublespinbox : input_list)
  // connect(doublespinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](const auto) {
  //   compute();
  // });
  /// Update values when frenet or cartesian plot is clicked
  connect(fcplot, &QCustomPlot::mousePress, [&](const QMouseEvent * event) {
    bool convert_to_frenet = false;
    double graph_x = NAN;
    double graph_y = NAN;
    if (plotter_->inCartesianRect(event->pos())) {
      plotter_->pixelToCartesian(event->pos(), graph_x, graph_y);
      if (convert_to_frenet) {
        // const auto frenet = ;
      }
    } else if (plotter_->inFrenetRect(event->pos())) {
      plotter_->pixelToFrenet(event->pos(), graph_x, graph_y);
    } else {
      return;  // click is outside of the axis rectangles
    }
    switch (event->button()) {
      case Qt::MiddleButton:
        ui_->lon_pos->setValue(graph_x);
        ui_->lat_pos->setValue(graph_y);
        break;
      case Qt::RightButton:
        ui_->lon_pos_2->setValue(graph_x);
        ui_->lat_pos_2->setValue(graph_y);
        break;
      default:;
    }
  });
  */
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
  // Raw path
  QVector<double> xs;
  QVector<double> ys;
  xs.reserve(static_cast<int>(raw_path.points.size()));
  ys.reserve(static_cast<int>(raw_path.points.size()));
  for (const auto & p : raw_path.points) {
    xs.push_back(p.pose.position.x);
    ys.push_back(p.pose.position.y);
  }
  output_pos_raw_path_curve_->setData(xs, ys);
  input_path_raw_curve_->setData(xs, ys);
  // Smooth path
  xs.clear();
  ys.clear();
  for (auto s = spline_path.firstS(); s <= spline_path.lastS(); s += 0.1) {
    const auto p = spline_path.cartesian(s);
    xs.push_back(p.x());
    ys.push_back(p.y());
  }
  input_path_smooth_curve_->setData(xs, ys);
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
    table->setItem(i, 1, new QTableWidgetItem(tr("%1").arg(candidate.valid)));
    auto * item = new QTableWidgetItem();
    item->setData(Qt::ItemDataRole::DisplayRole, qVariantFromValue(candidate.cost));
    table->setItem(i, 2, item);
  }
  table->setSortingEnabled(true);
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
  // TODO(Maxime CLEMENT): uncomment once jerk is calculated
  // ui_->output_jerk->graph(0)->setData(QVector<double>::fromStdVector(trajectory.times),
  // QVector<double>::fromStdVector(trajectory.jerks));
  ui_->cand_pos->rescaleAxes();
  ui_->cand_pos->replot();
  ui_->cand_vel->rescaleAxes();
  ui_->cand_vel->replot();
  ui_->cand_acc->rescaleAxes();
  ui_->cand_acc->replot();
  ui_->cand_jerk->rescaleAxes();
  ui_->cand_jerk->replot();
}

}  // namespace sampler_node::gui
