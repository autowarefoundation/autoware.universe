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
#ifndef CUSTOM_BUTTON_HPP_
#define CUSTOM_BUTTON_HPP_

#include <QColor>
#include <QFontMetrics>
#include <QGraphicsDropShadowEffect>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>

class CustomElevatedButton : public QPushButton
{
  Q_OBJECT

public:
  explicit CustomElevatedButton(
    const QString & text, const QColor & bgColor = QColor("#171C1F"),
    const QColor & textColor = QColor("#8bd0f0"), QWidget * parent = nullptr);
  void updateStyle(
    const QString & text, const QColor & bgColor, const QColor & textColor,
    const QColor & hoverColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  void enterEvent(QEvent * event) override;
  void leaveEvent(QEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QColor backgroundColor = QColor("##171C1F");
  QColor textColor = QColor("#8bd0f0");
  QColor hoverColor = QColor("#3C3F41");
  bool isHovered = false;
};

#endif  // CUSTOM_BUTTON_HPP_
