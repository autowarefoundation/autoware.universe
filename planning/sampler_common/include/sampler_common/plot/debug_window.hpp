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

#ifndef SAMPLER_COMMON__PLOT__DEBUG_WINDOW_HPP
#define SAMPLER_COMMON__PLOT__DEBUG_WINDOW_HPP

#include "sampler_common/plot/plotter.hpp"
#include "sampler_common/structures.hpp"

#include <QInputDialog>
#include <QMainWindow>

#include <qcustomplot.h>

#include <memory>
#include <vector>

namespace Ui
{
class MainWindow;
}  // namespace Ui

namespace sampler_common::plot
{
class MainWindow : public QMainWindow
{
  // Q_OBJECT

public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override = default;

  // @brief replot all curves
  void replot();
  // @brief set values displayed in the status bar
  void setStatus(const size_t nb_paths, const double compute_time_ms, const double plot_time_ms);

  // TODO make this private and have a better interface (separate class for interactive plot case ?)
  // Plotter to manage QCustomPlot of the Gui
  std::shared_ptr<Plotter> plotter_;

private slots:

private:
  // Gui
  std::shared_ptr<Ui::MainWindow> ui = std::make_shared<Ui::MainWindow>();
};
}  // namespace sampler_common::plot

#endif  // SAMPLER_COMMON__PLOT__DEBUG_WINDOW_HPP
