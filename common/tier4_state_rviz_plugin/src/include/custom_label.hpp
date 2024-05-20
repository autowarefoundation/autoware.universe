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
#ifndef CUSTOM_LABEL_HPP_
#define CUSTOM_LABEL_HPP_

#include <QLabel>
#include <QWidget>

#include <qcolor.h>

class CustomLabel : public QLabel
{
  Q_OBJECT

public:
  explicit CustomLabel(const QString & text, QWidget * parent = nullptr);
  void updateStyle(const QString & text, const QColor & bgColor, const QColor & textColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QColor backgroundColor = QColor("#171C1F");
  QColor textColor = QColor("#FFFFFF");
};

#endif  // CUSTOM_LABEL_HPP_
