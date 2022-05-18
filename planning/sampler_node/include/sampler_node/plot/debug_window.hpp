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

#ifndef SAMPLER_NODE__PLOT__DEBUG_WINDOW_HPP_
#define SAMPLER_NODE__PLOT__DEBUG_WINDOW_HPP_

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_node/plot/plotter.hpp"

#include <QInputDialog>
#include <QMainWindow>

#include <qcustomplot.h>

#include <memory>
#include <vector>

namespace Ui
{
class MainWindow;
}  // namespace Ui

namespace sampler_node::plot
{
class MainWindow : public QMainWindow
{
public:
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

  // @brief replot the trajectories
  void replot();
  // @brief set values displayed in the status bar
  void setStatus(const size_t nb_trajs, const double compute_time_ms, const double plot_time_ms);

  // TODO make this private and have a better interface (separate class for interactive plot case ?)
  // Plotter to manage QCustomPlot of the Gui
  std::shared_ptr<sampler_node::plot::Plotter> plotter_;

private slots:

private:
  // @brief compute trajectories for the current parameters
  void compute();

  // Gui
  Ui::MainWindow * ui;
};
}  // namespace sampler_node::plot

#endif  // SAMPLER_NODE__PLOT__DEBUG_WINDOW_HPP_
