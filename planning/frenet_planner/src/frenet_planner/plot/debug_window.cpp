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

#include "frenet_planner/plot/debug_window.hpp"

#include "frenet_planner/frenet_planner.hpp"
#include "frenet_planner/plot/ui_mainwindow.h"
#include "frenet_planner/transform/spline_transform.hpp"

#include <qcustomplot.h>
#include <qlineedit.h>
#include <qnamespace.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qtabwidget.h>
#include <qtextedit.h>

#include <cmath>

namespace frenet_planner::plot
{
MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  const auto & fcplot = ui->frenet_cartesian_plot;
  const auto & polyplot = ui->frenet_polynomials_plot;
  plotter_ = std::make_shared<Plotter>(fcplot, polyplot);

  // Signals
  /// Replot when tab is changed
  connect(ui->tabWidget, &QTabWidget::currentChanged, [&](const auto) { replot(); });
  /// Generate trajectories when button is clicked
  connect(ui->autoscale_toggle_button, &QPushButton::clicked, [&]() { replot(); });
  /// Regenerate trajectories whenever the inputs are changed
  auto input_list = ui->initial_states_box->findChildren<QDoubleSpinBox *>();
  input_list.append(ui->target_states_box->findChildren<QDoubleSpinBox *>());
  for (const auto * doublespinbox : input_list)
    connect(doublespinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](const auto) {
      compute();
    });
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
        ui->lon_pos->setValue(graph_x);
        ui->lat_pos->setValue(graph_y);
        break;
      case Qt::RightButton:
        ui->lon_pos_2->setValue(graph_x);
        ui->lat_pos_2->setValue(graph_y);
        break;
      default:;
    }
  });

  // Init
  compute();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::setStatus(
  const size_t nb_trajs, const double compute_time_ms, const double plot_time_ms)
{
  ui->statusBar->showMessage(
    QString("Generated Trajectories: %1 | Compute time: %2ms | Render time %3ms")
      .arg(
        QString::number(nb_trajs), QString::number(compute_time_ms),
        QString::number(plot_time_ms)));
}

void MainWindow::compute()
{
  std::chrono::steady_clock::time_point calc_begin = std::chrono::steady_clock::now();
  // random reference path
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<Point> reference_path;
  for (double x = 0.0; x <= 100.0; x += 1.0) {
    xs.push_back(x);
    ys.push_back(std::sin(x / 10) * 10);
    reference_path.emplace_back(xs.back(), ys.back());
  }
  frenet_planner::transform::Spline2D reference_spline(xs, ys);
  // generation parameters
  frenet_planner::FrenetState initial_state;
  initial_state.position.s = ui->lon_pos->value();
  initial_state.position.d = ui->lat_pos->value();
  initial_state.longitudinal_velocity = ui->lon_vel->value();
  initial_state.longitudinal_acceleration = ui->lon_acc->value();
  initial_state.lateral_velocity = ui->lat_vel->value();
  initial_state.lateral_acceleration = ui->lat_acc->value();
  frenet_planner::FrenetState target_state;
  target_state.position.s = ui->lon_pos_2->value();
  target_state.position.d = ui->lat_pos_2->value();
  target_state.longitudinal_velocity = ui->lon_vel_2->value();
  target_state.longitudinal_acceleration = ui->lon_acc_2->value();
  target_state.lateral_velocity = ui->lat_vel_2->value();
  target_state.lateral_acceleration = ui->lat_acc_2->value();
  /*
  frenet_planner::SamplingParameters sampling_parameters;
  sampling_parameters.min_duration = 5.0;
  sampling_parameters.max_duration = 50.0;
  sampling_parameters.duration_step = 5.0;
  sampling_parameters.time_resolution = 0.5;
  sampling_parameters.lateral_deviation_step = 0.5;
  sampling_parameters.min_lateral_deviation = -2.0;
  sampling_parameters.max_lateral_deviation = 2.0;
  sampling_parameters.target_velocity = 3.0;
  */
  // generate trajectories
  constexpr double duration = 10;
  constexpr double time_resolution = 0.1;
  std::vector<frenet_planner::Trajectory> trajs = {
    frenet_planner::generateCandidate(initial_state, target_state, duration, time_resolution)};
  // frenet_planner::generateCandidates(initial_state, sampling_parameters);
  frenet_planner::calculateCartesian(reference_spline, trajs);

  std::chrono::steady_clock::time_point calc_end = std::chrono::steady_clock::now();

  // plot
  std::chrono::steady_clock::time_point plot_begin = calc_end;
  plotter_->plotReferencePath(reference_path);
  plotter_->plotTrajectories(trajs);
  replot();
  std::chrono::steady_clock::time_point plot_end = std::chrono::steady_clock::now();
  setStatus(
    trajs.size(),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(calc_end - calc_begin).count()),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_begin).count()));
}

void MainWindow::replot()
{
  if (ui->tabWidget->currentWidget() == ui->frenet_cartesian_plot) {
    plotter_->replot(true, false, ui->autoscale_toggle_button->isChecked());
  } else if (ui->tabWidget->currentWidget() == ui->frenet_polynomials_plot) {
    plotter_->replot(false, true, ui->autoscale_toggle_button->isChecked());
  }
}

}  // namespace frenet_planner::plot
