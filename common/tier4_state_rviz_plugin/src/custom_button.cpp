// Copyright 2024 The Autoware Contributors
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
#include "include/custom_button.hpp"

#include "src/include/material_colors.hpp"

CustomElevatedButton::CustomElevatedButton(
  const QString & text, const QColor & bgColor, const QColor & textColor, const QColor & hoverColor,
  const QColor & disabledBgColor, const QColor & disabledTextColor, QWidget * parent)
: QPushButton(text, parent),
  backgroundColor(bgColor),
  textColor(textColor),
  hoverColor(hoverColor),
  disabledBgColor(disabledBgColor),
  disabledTextColor(disabledTextColor)
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
  setCursor(Qt::PointingHandCursor);

  // set font weight to bold and size to 12
  QFont font = this->font();
  font.setWeight(QFont::Bold);
  font.setFamily("Roboto");
  setFont(font);

  // Set up drop shadow effect
  QGraphicsDropShadowEffect * shadowEffect = new QGraphicsDropShadowEffect(this);
  shadowEffect->setBlurRadius(15);
  shadowEffect->setOffset(3, 3);
  shadowEffect->setColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.shadow.c_str()));
  setGraphicsEffect(shadowEffect);
}

QSize CustomElevatedButton::sizeHint() const
{
  QFontMetrics fm(font());
  int textWidth = fm.horizontalAdvance(text());
  int textHeight = fm.height();
  int width = textWidth + 45;    // Adding padding
  int height = textHeight + 25;  // Adding padding
  return QSize(width, height);
}

// cppcheck-suppress unusedFunction
void CustomElevatedButton::updateStyle(
  const QString & text, const QColor & bgColor, const QColor & textColor, const QColor & hoverColor,
  const QColor & disabledBgColor, const QColor & disabledTextColor)
{
  setText(text);
  backgroundColor = bgColor;
  this->textColor = textColor;
  this->hoverColor = hoverColor;
  this->disabledBgColor = disabledBgColor;
  this->disabledTextColor = disabledTextColor;
  update();  // Force repaint
}

void CustomElevatedButton::enterEvent(QEvent * event)
{
  isHovered = true;
  update();
  QPushButton::enterEvent(event);
}

void CustomElevatedButton::leaveEvent(QEvent * event)
{
  isHovered = false;
  update();
  QPushButton::leaveEvent(event);
}
