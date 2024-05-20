#include "include/custom_toggle_switch.hpp"

#include <QDebug>

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

void CustomToggleSwitch::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  int margin = 2;
  int circleRadius = height() / 2 - margin * 2;
  QRect r = rect().adjusted(margin, margin, -margin, -margin);
  bool isChecked = this->isChecked();

  QRect borderR = r.adjusted(-margin, -margin, margin, margin);
  p.setBrush(isChecked ? QColor("#8bd0f0") : QColor("#303538"));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(borderR, circleRadius + 4, circleRadius + 4);

  p.setBrush(isChecked ? QColor("#8bd0f0") : QColor("#303538"));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(r, circleRadius + 4, circleRadius + 4);

  int minX = r.left() + margin * 2;
  int maxX = r.right() - circleRadius * 2 - margin;
  int circleX = isChecked ? maxX : minX;
  QRect circleRect(circleX, r.top() + margin, circleRadius * 2, circleRadius * 2);
  p.setBrush(isChecked ? QColor("#003546") : QColor("#8a9297"));
  p.drawEllipse(circleRect);
}

void CustomToggleSwitch::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    qDebug() << "Mouse release event at x:" << event->pos().x() << "y:" << event->pos().y();

    int middleX = rect().center().x();            // Calculate the middle of the switch
    bool newState = event->pos().x() >= middleX;  // Determine new state based on click position

    setCheckedState(newState);
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