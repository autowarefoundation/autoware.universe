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
#include "include/custom_segmented_button_item.hpp"

CustomSegmentedButtonItem::CustomSegmentedButtonItem(const QString & text, QWidget * parent)
: QPushButton(text, parent),
  bgColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.surface_container_low.c_str())),
  checkedBgColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.secondary_container.c_str())),
  inactiveTextColor(QColor(autoware::state_rviz_plugin::colors::default_colors.on_surface.c_str())),
  activeTextColor(
    QColor(autoware::state_rviz_plugin::colors::default_colors.on_secondary_container.c_str())),
  isHovered(false),
  isActivated(false),
  isDisabled(false)

{
  setCheckable(true);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setCursor(Qt::PointingHandCursor);
}

// void CustomSegmentedButtonItem::updateSize()
// {
//   QFontMetrics fm(font());
//   int width = fm.horizontalAdvance(text()) + 40;  // Add padding
//   int height = fm.height() + 20;                  // Add padding
//   setFixedSize(width, height);
// }

// cppcheck-suppress unusedFunction
void CustomSegmentedButtonItem::setHovered(bool hovered)
{
  isHovered = hovered;
  updateCheckableState();
}

// cppcheck-suppress unusedFunction
void CustomSegmentedButtonItem::setCheckableButton(bool checkable)
{
  setCheckable(checkable);
  updateCheckableState();
}

void CustomSegmentedButtonItem::updateCheckableState()
{
  setCursor(
    isDisabled ? Qt::ForbiddenCursor
               : (isCheckable() ? Qt::PointingHandCursor : Qt::ForbiddenCursor));
  update();
}

// cppcheck-suppress unusedFunction
void CustomSegmentedButtonItem::setDisabledButton(bool disabled)
{
  isDisabled = disabled;
  updateCheckableState();
}

// cppcheck-suppress unusedFunction
void CustomSegmentedButtonItem::setActivated(bool activated)
{
  isActivated = activated;
  update();
}

void CustomSegmentedButtonItem::enterEvent(QEvent * event)
{
  if (isCheckable()) {
    isHovered = true;
    update();
  }
  QPushButton::enterEvent(event);
}

void CustomSegmentedButtonItem::leaveEvent(QEvent * event)
{
  if (isCheckable()) {
    isHovered = false;
    update();
  }
  QPushButton::leaveEvent(event);
}
