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
#include "include/custom_toggle_switch.hpp"

CustomToggleSwitch::CustomToggleSwitch(QWidget * parent) : QCheckBox(parent)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setCursor(Qt::PointingHandCursor);

  connect(this, &QCheckBox::stateChanged, this, [this]() {
    if (!blockSignalsGuard) {
      update();  // Force repaint
    }
  });
}

QSize CustomToggleSwitch::sizeHint() const
{
  return QSize(50, 30);  // Preferred size of the toggle switch
}

void CustomToggleSwitch::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    setCheckedState(!isChecked());
  }
  QCheckBox::mouseReleaseEvent(event);
}

void CustomToggleSwitch::setCheckedState(bool state)
{
  blockSignalsGuard = true;
  setChecked(state);
  blockSignalsGuard = false;
  update();  // Force repaint
}
