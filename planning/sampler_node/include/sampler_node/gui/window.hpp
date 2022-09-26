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

#ifndef SAMPLER_NODE__GUI__WINDOW_HPP_
#define SAMPLER_NODE__GUI__WINDOW_HPP_

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <QMainWindow>

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <qcustomplot.h>

#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}  // namespace Ui
QT_END_NAMESPACE

namespace sampler_node::gui
{
class MainWindow : public QMainWindow
{
private:
  Ui::MainWindow * ui_;
  // Manually managed plots
  //// Output tab
  QCPCurve * output_pos_curve_;
  QCPItemLine * output_pos_arrow_;
  QCPCurve * output_pos_raw_path_curve_;
  //// Input tab
  QCPCurve * input_path_raw_curve_;
  QCPCurve * input_path_smooth_curve_;
  QCPItemLine * input_path_arrow_;
  //// Candidate tab
  QCPCurve * cand_pos_curve_;
  // Cached data
  std::vector<sampler_common::Trajectory> candidates_;

public:
  enum Tab { selected = 0, inputs, candidates, frenet, bezier, pruning, selection, perf, SIZE };
  explicit MainWindow(QWidget * parent = nullptr);
  ~MainWindow() override;

  void update();
  // @brief replot the trajectories
  void replot();
  // @brief set values displayed in the status bar
  void setStatus(const size_t nb_trajs, const double compute_time_ms, const double plot_time_ms);

  void plotSelected(const sampler_common::Trajectory & trajectory);
  void plotInputs(
    const autoware_auto_planning_msgs::msg::Path & raw_reference_path_,
    const sampler_common::transform::Spline2D & reference_path_,
    const sampler_common::Configuration & configuration_);
  void fillCandidatesTable(const std::vector<sampler_common::Trajectory> & candidates);
  void plotCandidate(const sampler_common::Trajectory & trajectory);

private slots:
};
}  // namespace sampler_node::gui

#endif  // SAMPLER_NODE__GUI__WINDOW_HPP_
