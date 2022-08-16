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

#include "string_overlay_display.hpp"

#include <QPainter>
#include <QStaticText>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <rviz_common/uniform_string_stream.hpp>

namespace rviz_plugins
{
StringOverlayDisplay::StringOverlayDisplay()
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

  property_fg_color_ = new rviz_common::properties::ColorProperty("Foreground Color", QColor(255, 255, 40), "Foreground Color", this, SLOT(updateVisualization()), this);
  property_bg_color_ = new rviz_common::properties::ColorProperty("Background Color", QColor(50, 50, 50), "Background Color", this, SLOT(updateVisualization()), this);

  property_fg_alpha_ = new rviz_common::properties::FloatProperty("Foregrund Alpha", 0.8, "Foreground Alpha", this, SLOT(updateVisualization()), this);
  property_bg_alpha_ = new rviz_common::properties::FloatProperty("Background Alpha", 0.2, "Background Alpha", this, SLOT(updateVisualization()), this);
  property_fg_alpha_->setMin(0.0);
  property_fg_alpha_->setMax(1.0);
  property_bg_alpha_->setMin(0.0);
  property_bg_alpha_->setMax(1.0);

  property_font_size_ = new rviz_common::properties::IntProperty("Font Size", 12, "Font Size", this, SLOT(updateVisualization()));
  property_font_size_->setMin(0);
}

StringOverlayDisplay::~StringOverlayDisplay()
{
  if (initialized()) {
    overlay_->hide();
  }
}

void StringOverlayDisplay::onInitialize()
{
  static int count = 0;

  rviz_common::UniformStringStream ss;
  ss << "StringOverlayDisplay" << count++;
  auto logger = context_->getRosNodeAbstraction().lock()->get_raw_node()->get_logger();
  overlay_.reset(new jsk_rviz_plugins::OverlayObject(scene_manager_, logger, ss.str()));

  overlay_->show();

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
}

void StringOverlayDisplay::onEnable()
{
  subscribe();
  overlay_->show();
}

void StringOverlayDisplay::onDisable()
{
  unsubscribe();
  reset();
  overlay_->hide();
}

void StringOverlayDisplay::subscribe()
{
  topic_name_ = property_topic_name_->getStdString();
  if (topic_name_.length() > 0 && topic_name_ != "/") {
    rclcpp::Node::SharedPtr raw_node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    string_sub_ = raw_node->create_subscription<std_msgs::msg::String>(topic_name_, 10, std::bind(&StringOverlayDisplay::processMessage, this, std::placeholders::_1));
  }
}

void StringOverlayDisplay::unsubscribe()
{
  string_sub_.reset();
}

void StringOverlayDisplay::processMessage(const std_msgs::msg::String::ConstSharedPtr msg_ptr)
{
  if (!isEnabled()) return;

  last_msg_ptr_ = msg_ptr;

  queueRender();
}

void StringOverlayDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::string current_topic_name = property_topic_name_->getStdString();
  if (current_topic_name != topic_name_) {
    topic_name_ = current_topic_name;
    subscribe();
  }

  updateVisualization();
}

void StringOverlayDisplay::updateVisualization()
{
  if (!last_msg_ptr_) return;

  QColor background_color(property_bg_color_->getColor());
  background_color.setAlpha(255 * property_bg_alpha_->getFloat());
  jsk_rviz_plugins::ScopedPixelBuffer buffer = overlay_->getBuffer();
  hud_ = buffer.getQImage(*overlay_);
  hud_.fill(background_color);

  overlay_->updateTextureSize(property_width_->getInt(), property_height_->getInt());
  overlay_->setPosition(property_left_->getInt(), property_top_->getInt());
  overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());

  const int w = overlay_->getTextureWidth();
  const int h = overlay_->getTextureHeight();

  const int font_size_ = property_font_size_->getInt();
  const bool align_bottom_ = false;
  const std::string text_ = last_msg_ptr_->data;


  QPainter painter(&hud_);
  QColor fg_color_(property_fg_color_->getColor());
  fg_color_.setAlpha(255 * property_fg_alpha_->getFloat());

  if (font_size_ != 0) {
    QFont font("Liberation Sans");
    font.setPointSize(font_size_);
    font.setBold(true);
    painter.setFont(font);
  }
  if (text_.length() > 0) {
    std::string color_wrapped_text
        = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
            % text_ % fg_color_.red() % fg_color_.green() % fg_color_.blue() % fg_color_.alpha())
              .str();

    QStaticText static_text(boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());

    static_text.setTextWidth(w);
    if (!align_bottom_) {
      painter.drawStaticText(0, 0, static_text);
    } else {
      QStaticText only_wrapped_text(color_wrapped_text.c_str());
      QFontMetrics fm(painter.fontMetrics());
      QRect text_rect = fm.boundingRect(0, 0, w, h,
          Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
          only_wrapped_text.text().remove(QRegExp("<[^>]*>")));
      painter.drawStaticText(0, h - text_rect.height(), static_text);
    }
  }
  painter.end();
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::StringOverlayDisplay, rviz_common::Display)
