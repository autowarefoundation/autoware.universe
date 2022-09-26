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

#ifndef SAMPLER_NODE__GUI__GUI_HPP_
#define SAMPLER_NODE__GUI__GUI_HPP_

#include "frenet_planner/structures.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"
#include "sampler_node/gui/window.hpp"

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <iostream>
#include <string>
#include <vector>

namespace sampler_node::gui
{
class GUI
{
  using Tab = MainWindow::Tab;
  // QT App
  int argc_ = 1;
  std::vector<char *> argv_ = {std::string("Sampling Planner GUI").data()};
  QApplication qapplication_;
  MainWindow window_;
  // Data
  sampler_common::Configuration configuration_;
  autoware_auto_planning_msgs::msg::Path raw_reference_path_;
  sampler_common::transform::Spline2D reference_path_;
  sampler_common::Constraints constraints_;
  std::vector<sampler_common::Trajectory> candidates_;
  std::optional<size_t> selected_id_;
  std::optional<sampler_common::Trajectory> previous_selected_;
  //
  std::array<bool, MainWindow::Tab::SIZE> to_update_{};

public:
  GUI() : qapplication_(argc_, argv_.data()) { window_.show(); }

  void setInputs(
    const autoware_auto_planning_msgs::msg::Path & path,
    const sampler_common::transform::Spline2D & spline,
    const sampler_common::Configuration & config)
  {
    raw_reference_path_ = path;
    reference_path_ = spline;
    configuration_ = config;
    to_update_[Tab::inputs] = true;
  }

  void setConstraints(const sampler_common::Constraints & constraints)
  {
    this->constraints_ = constraints;
    to_update_[Tab::inputs] = true;
  }
  // void setParameters();
  void setOutputs(
    const std::vector<sampler_common::Trajectory> & candidates,
    const std::optional<size_t> & selected_id,
    const std::optional<sampler_common::Trajectory> & previous_selected)
  {
    candidates_ = candidates;
    selected_id_ = selected_id;
    previous_selected_ = previous_selected;
    to_update_[Tab::selected] = true;
    to_update_[Tab::candidates] = true;
  }
  void setFocus(size_t id)
  {
    (void)id;
    to_update_[Tab::candidates];
  }
  void setFrenetTrajectories(
    const std::vector<frenet_planner::Trajectory> & trajectories,
    const sampler_common::Trajectory & base = {})
  {
    // TODO(Maxime CLEMENT): GUI for frenet trajectories
    (void)trajectories;
    (void)base;
    to_update_[Tab::frenet] = true;
  }
  void setPerformances(
    const size_t nb_trajs, const double compute_time_ms, const double plot_time_ms)
  {
    window_.setStatus(nb_trajs, compute_time_ms, plot_time_ms);
  }

  void unsetFocus() { to_update_[Tab::candidates]; }
  void update()
  {
    if (to_update_[Tab::selected]) {
      if (selected_id_) {
        window_.plotSelected(candidates_[*selected_id_]);
      }
      to_update_[Tab::selected] = false;
    }
    if (to_update_[Tab::inputs]) {
      window_.plotInputs(raw_reference_path_, reference_path_, configuration_);
      to_update_[Tab::inputs] = false;
    }
    if (to_update_[Tab::candidates]) {
      window_.fillCandidatesTable(candidates_);
      to_update_[Tab::candidates] = false;
    }
    if (to_update_[Tab::frenet]) {
      to_update_[Tab::frenet] = false;
    }
    window_.update();
  }
};
}  // namespace sampler_node::gui
#endif  // SAMPLER_NODE__GUI__GUI_HPP_
