// CustomToggleSwitch.cpp
#include "custom_toggle_switch.hpp"

#include <QMouseEvent>
#include <QPainter>

CustomToggleSwitch::CustomToggleSwitch(QWidget * parent) : QCheckBox(parent)
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setCursor(Qt::PointingHandCursor);
}

QSize CustomToggleSwitch::sizeHint() const
{
  return QSize(50, 20);  // Preferred size of the toggle switch
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

  // Draw Border that is larger than the background with a bit of thickness
  QRect borederR = r.adjusted(-margin, -margin, margin, margin);
  p.setPen(QPen(!isChecked ? QColor("#00E678") : QColor("#525252"), 2));
  p.setBrush(Qt::NoBrush);
  p.drawRoundedRect(borederR, circleRadius + 4, circleRadius + 4);

  // Draw background
  p.setBrush(isChecked ? QColor("#00E678") : QColor("#525252"));
  p.setPen(Qt::NoPen);
  p.drawRoundedRect(r, circleRadius + 4, circleRadius + 4);

  // Draw indicator
  int circleX = isChecked ? (r.right() - circleRadius * 2 - margin) : r.left() + margin;
  QRect circleRect(circleX, r.top() + margin, circleRadius * 2, circleRadius * 2);
  p.setBrush(QColor("#FFF"));
  p.drawEllipse(circleRect);
}

void CustomToggleSwitch::mousePressEvent(QMouseEvent * event)
{
  if (event->buttons() & Qt::LeftButton) {
    isDragging = true;
    dragStartPoint = event->pos();
  }
  QCheckBox::mousePressEvent(event);
}

void CustomToggleSwitch::mouseReleaseEvent(QMouseEvent * event)
{
  if (isDragging) {
    if (qAbs(event->pos().x() - dragStartPoint.x()) < 10) {  // Simple click
      setChecked(!isChecked());
    }
    isDragging = false;
  }
  QCheckBox::mouseReleaseEvent(event);
}
