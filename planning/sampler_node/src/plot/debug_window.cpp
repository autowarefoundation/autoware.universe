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

#include "sampler_node/plot/debug_window.hpp"

#include "frenet_planner/frenet_planner.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/plot/ui_mainwindow.h"

#include <qcustomplot.h>
#include <qlineedit.h>
#include <qnamespace.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qtabwidget.h>
#include <qtextedit.h>

#include <cmath>

namespace sampler_node::plot
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

void MainWindow::replot()
{
  if (ui->tabWidget->currentWidget() == ui->frenet_cartesian_plot) {
    plotter_->replot(true, false, ui->autoscale_toggle_button->isChecked());
  } else if (ui->tabWidget->currentWidget() == ui->frenet_polynomials_plot) {
    plotter_->replot(false, true, ui->autoscale_toggle_button->isChecked());
  }
}

}  // namespace sampler_node::plot
