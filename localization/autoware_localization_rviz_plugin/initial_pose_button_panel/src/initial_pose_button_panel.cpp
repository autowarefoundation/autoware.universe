/*
 * Copyright (c) 2018, TierIV Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <algorithm>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "QFileDialog"
#include "QHBoxLayout"
#include "QLineEdit"
#include "QMap"
#include "QPainter"
#include "QPushButton"
#include "QSignalMapper"
#include "QStandardPaths"
#include "QTimer"
#include "QVBoxLayout"

#include "rviz_common/display_context.hpp"
#include "initial_pose_button_panel.hpp"

namespace autoware_localization_rviz_plugin
{
InitialPoseButtonPanel::InitialPoseButtonPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  topic_label_ = new QLabel("PoseWithCovarianceStamped ");
  topic_label_->setAlignment(Qt::AlignCenter);

  topic_edit_ = new QLineEdit("/sensing/gnss/pose_with_covariance");
  connect(topic_edit_, SIGNAL(textEdited(QString)), SLOT(editTopic()));

  initialize_button_ = new QPushButton("Wait for subscribe topic");
  initialize_button_->setEnabled(false);
  connect(initialize_button_, SIGNAL(clicked(bool)), SLOT(pushInitializeButton()));

  status_label_ = new QLabel("Not Initialze");
  status_label_->setAlignment(Qt::AlignCenter);
  status_label_->setStyleSheet("QLabel { background-color : gray;}");

  QSizePolicy * q_size_policy = new QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  initialize_button_->setSizePolicy(*q_size_policy);

  QHBoxLayout * topic_layout = new QHBoxLayout;
  topic_layout->addWidget(topic_label_);
  topic_layout->addWidget(topic_edit_);

  QVBoxLayout * v_layout = new QVBoxLayout;
  v_layout->addLayout(topic_layout);
  v_layout->addWidget(initialize_button_);
  v_layout->addWidget(status_label_);

  setLayout(v_layout);

  rclcpp::Node::SharedPtr raw_node = 
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  pose_cov_sub_ = raw_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
    (topic_edit_->text().toStdString(), 10, 
    std::bind(&InitialPoseButtonPanel::callbackPoseCov, this, std::placeholders::_1));

  client_ = 
    raw_node->create_client<autoware_localization_srvs::srv::PoseWithCovarianceStamped>(
    "/localization/util/pose_initializer_srv");
}

void InitialPoseButtonPanel::callbackPoseCov(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  pose_cov_msg_ = *msg;
  initialize_button_->setText("Pose Initializer Let's GO!");
  initialize_button_->setEnabled(true);
}

void InitialPoseButtonPanel::editTopic()
{
  pose_cov_sub_.reset();
  rclcpp::Node::SharedPtr raw_node = 
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pose_cov_sub_ = raw_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
    (topic_edit_->text().toStdString(), 10, 
    std::bind(&InitialPoseButtonPanel::callbackPoseCov, this, std::placeholders::_1));
  initialize_button_->setText("Wait for subscribe topic");
  initialize_button_->setEnabled(false);
}

void InitialPoseButtonPanel::pushInitializeButton()
{
  // lock button
  initialize_button_->setEnabled(false);

  status_label_->setStyleSheet("QLabel { background-color : dodgerblue;}");
  status_label_->setText("Initializing...");

  std::thread thread([this] {
    auto req =
      std::make_shared<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request>();
    req->pose_with_cov = pose_cov_msg_;

    client_->async_send_request(
      req, 
      [this](rclcpp::Client<autoware_localization_srvs::srv::PoseWithCovarianceStamped>::SharedFuture
        result){
          status_label_->setStyleSheet("QLabel { background-color : lightgreen;}");
          status_label_->setText("OK!!!");
          
          // unlock button
          initialize_button_->setEnabled(true);
    });

  });

  thread.detach();
}

}  // end namespace autoware_localization_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autoware_localization_rviz_plugin::InitialPoseButtonPanel, rviz_common::Panel)

