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

CustomElevatedButton::CustomElevatedButton(
  const QString & text, const QColor & bgColor, const QColor & textColor, QWidget * parent)
: QPushButton(text, parent), backgroundColor(bgColor), textColor(textColor)
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
  shadowEffect->setColor(QColor(0, 0, 0, 80));
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

QSize CustomElevatedButton::minimumSizeHint() const
{
  return sizeHint();
}

void CustomElevatedButton::updateStyle(
  const QString & text, const QColor & bgColor, const QColor & textColor, const QColor & hoverColor)
{
  setText(text);
  backgroundColor = bgColor;
  this->textColor = textColor;
  this->hoverColor = hoverColor;
  update();  // Force repaint
}

void CustomElevatedButton::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  QStyleOption opt;
  opt.initFrom(this);
  QRect r = rect();

  // Determine the button's color based on its state
  QColor buttonColor;
  if (isHovered && isEnabled()) {
    buttonColor = hoverColor;
  } else {
    buttonColor = backgroundColor;
  }

  int cornerRadius = height() / 2;  // Making the corner radius proportional to the height

  // Draw button background
  QPainterPath backgroundPath;
  backgroundPath.addRoundedRect(r, cornerRadius, cornerRadius);
  painter.setBrush(buttonColor);
  painter.setPen(Qt::NoPen);
  painter.drawPath(backgroundPath);

  // Draw button text
  painter.setPen(textColor);
  painter.drawText(r, Qt::AlignCenter, text());
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
