// Copyright 2021 Tier IV, Inc.
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

#ifndef SCREEN_CAPTURE_PANEL_HPP_
#define SCREEN_CAPTURE_PANEL_HPP_

// Qt
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>

// rviz
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#include <opencv2/opencv.hpp>

#include <memory>
#include <string>

class QLineEdit;

class AutowareScreenCapturePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit AutowareScreenCapturePanel(QWidget * parent = nullptr);
  void update();
  void onInitialize() override;
  void createWallTimer();
  void onTimer();
  void convertPNGImagesToMP4();

public Q_SLOTS:
  void onClickScreenCapture();
  void onClickCaptureToVideo();
  void onClickVideoCapture();

private:
  QLabel * ros_time_label_;
  QPushButton * screen_capture_button_ptr_;
  QPushButton * capture_to_mp4_button_ptr_;
  QSpinBox * capture_hz_;
  QTimer * capture_timer_;
  enum class State { WAITING_FOR_CAPTURE, CAPTURING, FINALIZED };
  State state_;
  std::string root_folder_;
  size_t counter_;

protected:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

#endif  // SCREEN_CAPTURE_PANEL_HPP_
