// Copyright 2020 Tier IV, Inc.
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

#include "image_overlay_display.hpp"

#include <QPainter>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/highgui.hpp>
#include <rviz_common/uniform_string_stream.hpp>

namespace rviz_plugins
{
ImageOverlayDisplay::ImageOverlayDisplay()
{
  property_topic_name_ = new rviz_common::properties::StringProperty("Topic", "/", "String", this, SLOT(updateVisualization()));
  property_left_ = new rviz_common::properties::IntProperty("Left", 128, "Left of the plotter window", this, SLOT(updateVisualization()), this);
  property_left_->setMin(0);
  property_top_ = new rviz_common::properties::IntProperty("Top", 128, "Top of the plotter window", this, SLOT(updateVisualization()));
  property_top_->setMin(0);

  property_width_ = new rviz_common::properties::IntProperty("Width", 256, "Width of the plotter window", this, SLOT(updateVisualization()), this);
  property_width_->setMin(10);
  property_height_ = new rviz_common::properties::IntProperty("Height", 256, "Height of the plotter window", this, SLOT(updateVisualization()), this);
  property_height_->setMin(10);

  property_alpha_ = new rviz_common::properties::FloatProperty("Alpha", 0.8, "Foreground Alpha", this, SLOT(updateVisualization()), this);
  property_alpha_->setMin(0.0);
  property_alpha_->setMax(1.0);

  property_image_type_ = new rviz_common::properties::BoolProperty("Image Topic Style", true, "is compresed?", this, SLOT(updateVisualization()));
}

ImageOverlayDisplay::~ImageOverlayDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void ImageOverlayDisplay::onInitialize()
{
  static int count = 0;

  rviz_common::UniformStringStream ss;
  ss << "ImageOverlayDisplay" << count++;
  auto logger = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger();
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(scene_manager_, logger, ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
  it_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(raw_node));
}

void ImageOverlayDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void ImageOverlayDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void ImageOverlayDisplay::subscribe()
{
  topic_name_ = property_topic_name_->getStdString();
  std::cout << "try to subscribe " << topic_name_ << std::endl;
  if (topic_name_.length() > 0 && topic_name_ != "/") {
    sub_ = std::make_shared<image_transport::Subscriber>(it_->subscribe(topic_name_, 10, std::bind(&ImageOverlayDisplay::processMessage, this, std::placeholders::_1)));
  }
}

void ImageOverlayDisplay::unsubscribe()
{
  sub_.reset();
}

void ImageOverlayDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) return;

  std::cout << "processMessage" << std::endl;
  last_msg_ptr_ = msg_ptr;
  update_required_ = true;
  queueRender();
}

void ImageOverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::string current_topic_name = property_topic_name_->getStdString();
  if (current_topic_name != topic_name_) {
    topic_name_ = current_topic_name;
    subscribe();
  }

  if (update_required_) {
    updateVisualization();
    update_required_ = false;
  }
}

void ImageOverlayDisplay::updateVisualization()
{
  if (!last_msg_ptr_) return;
  if (last_msg_ptr_->width == 0 || last_msg_ptr_->height == 0) return;

  overlay_->updateTextureSize(last_msg_ptr_->width, last_msg_ptr_->height);
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(property_width_->getInt(), property_height_->getInt());

  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  QImage hud = buffer.getQImage(*overlay_);

  cv::Mat bgr_image;
  try {
    const cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_msg_ptr_, sensor_msgs::image_encodings::BGR8);
    bgr_image = cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }

  const int w = hud.width();
  const int h = hud.height();

  std::vector<cv::Mat> channels;
  cv::split(bgr_image, channels);
  const cv::Mat a_image(bgr_image.rows, bgr_image.cols, CV_8UC1, cv::Scalar(property_alpha_->getFloat() * 255.0));
  channels.push_back(a_image);

  cv::Mat bgra_image;
  cv::merge(channels, bgra_image);

  memcpy(hud.scanLine(0), bgra_image.data, w * h * bgra_image.elemSize());
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ImageOverlayDisplay, rviz_common::Display)
