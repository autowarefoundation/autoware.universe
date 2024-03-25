#include "custom_slider.hpp"

CustomSlider::CustomSlider(Qt::Orientation orientation, QWidget * parent)
: QSlider(orientation, parent)
{
  // Initialization code here (if necessary)
}

void CustomSlider::paintEvent(QPaintEvent * event)
{
  QSlider::paintEvent(event);  // Call base class paint event

  QPainter painter(this);
  QStyleOptionSlider opt;
  initStyleOption(&opt);

  QRect handleRect =
    style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);
  QPainterPath path;
  path.addEllipse(handleRect.center(), handleRect.width() / 2, handleRect.height() / 2);

  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillPath(path, QBrush(green));
}
