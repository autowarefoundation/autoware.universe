// CustomToggleSwitch.cpp
#include "custom_toggle_switch.hpp"

CustomToggleSwitch::CustomToggleSwitch(QWidget * parent) : QCheckBox(parent), isDragging(false)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setCursor(Qt::PointingHandCursor);
}

QSize CustomToggleSwitch::sizeHint() const
{
  return QSize(60, 30);  // Preferred size of the toggle switch
}

void CustomToggleSwitch::paintEvent(QPaintEvent *)
{
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  // Adjust these values based on your widget's size
  int margin = 2;                                // Margin around the toggle switch
  int circleRadius = height() / 2 - margin * 2;  // Circle radius based on the widget's height

  QRect r = rect().adjusted(margin, margin, -margin, -margin);
  bool isChecked = this->isChecked();

  // Draw border that is larger than the background with a bit of thickness
  QRect borderR = r.adjusted(-margin, -margin, margin, margin);
  p.setBrush(isChecked ? QColor("#8bd0f0") : QColor("#303538"));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(borderR, circleRadius + 4, circleRadius + 4);

  // Draw background
  p.setBrush(isChecked ? QColor("#8bd0f0") : QColor("#303538"));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(r, circleRadius + 4, circleRadius + 4);

  // Draw indicator
  int minX = r.left() + margin * 2;
  int maxX = r.right() - circleRadius * 2 - margin;
  int circleX =
    isDragging ? qBound(minX, dragStartPoint.x() - circleRadius, maxX) : (isChecked ? maxX : minX);
  QRect circleRect(circleX, r.top() + margin, circleRadius * 2, circleRadius * 2);
  p.setBrush(isChecked ? QColor("#003546") : QColor("#8a9297"));
  p.drawEllipse(circleRect);
}

void CustomToggleSwitch::mousePressEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    isDragging = true;
    dragStartPoint = event->pos();
  }
  QCheckBox::mousePressEvent(event);
}

void CustomToggleSwitch::mouseMoveEvent(QMouseEvent * event)
{
  if (isDragging) {
    dragStartPoint = event->pos();
    update();  // Force repaint
  }
  QCheckBox::mouseMoveEvent(event);
}

void CustomToggleSwitch::mouseReleaseEvent(QMouseEvent * event)
{
  if (isDragging) {
    if (rect().contains(event->pos())) {  // Ensure click is within the switch's bounds
      int minX = rect().left() + 2;
      int maxX = rect().right() - height() / 2 - 2;
      int middleX = (minX + maxX) / 2;
      bool newState = dragStartPoint.x() >= middleX;
      setChecked(newState);
    }
    isDragging = false;
    update();  // Force repaint
  }
  QCheckBox::mouseReleaseEvent(event);
}
