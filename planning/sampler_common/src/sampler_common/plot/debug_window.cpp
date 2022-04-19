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

#include "sampler_common/plot/debug_window.hpp"

#include "sampler_common/plot/ui_mainwindow.h"
#include "sampler_common/transform/spline_transform.hpp"

#include <qcustomplot.h>
#include <qlineedit.h>
#include <qnamespace.h>
#include <qpushbutton.h>
#include <qspinbox.h>
#include <qtabwidget.h>
#include <qtextedit.h>

#include <cmath>

namespace sampler_common::plot
{
MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent)
{
  ui->setupUi(this);

  const auto & main_plot = ui->frenet_cartesian_plot;
  plotter_ = std::make_shared<Plotter>(main_plot);

  // Signals
  connect(ui->tabWidget, &QTabWidget::currentChanged, [&](const auto) { replot(); });
  connect(ui->autoscale_toggle_button, &QPushButton::clicked, [&]() { replot(); });
}

void MainWindow::setStatus(
  const size_t nb_paths, const double compute_time_ms, const double plot_time_ms)
{
  ui->statusBar->showMessage(QString("Generated Paths: %1 | Compute time: %2ms | Render time %3ms")
                               .arg(
                                 QString::number(nb_paths), QString::number(compute_time_ms),
                                 QString::number(plot_time_ms)));
}

void MainWindow::replot() { plotter_->replot(ui->autoscale_toggle_button->isChecked()); }

}  // namespace sampler_common::plot
