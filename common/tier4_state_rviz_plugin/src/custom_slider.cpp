#include "custom_slider.hpp"

CustomSlider::CustomSlider(Qt::Orientation orientation, QWidget * parent)
: QSlider(orientation, parent)
{
  setMinimumHeight(40);  // Ensure there's enough space for the custom track
}

void CustomSlider::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);

  // Initialize style option
  QStyleOptionSlider opt;
  initStyleOption(&opt);

  QRect grooveRect =
    style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderGroove, this);
  int centerY = grooveRect.center().y();
  QRect handleRect =
    style()->subControlRect(QStyle::CC_Slider, &opt, QStyle::SC_SliderHandle, this);

  int trackThickness = 10;
  int gap = 8;

  QRect beforeRect(
    grooveRect.x(), centerY - trackThickness / 2, handleRect.center().x() - grooveRect.x() - gap,
    trackThickness);

  QRect afterRect(
    handleRect.center().x() + gap, centerY - trackThickness / 2,
    grooveRect.right() - handleRect.center().x() - gap, trackThickness);

  QColor inactiveTrackColor("#004d64");
  QColor activeTrackColor("#8bd0f0");

  // Manually create path for before track with asymmetric roundedness
  QPainterPath beforePath;
  beforePath.moveTo(beforeRect.left(), centerY + trackThickness / 2);  // Start from bottom-left

  // Left side with 50% roundedness
  beforePath.quadTo(
    beforeRect.left(), centerY - trackThickness / 2, beforeRect.left() + trackThickness * 0.5,
    centerY - trackThickness / 2);

  // Straight line to right side start
  beforePath.lineTo(beforeRect.right() - trackThickness * 0.2, centerY - trackThickness / 2);

  // Right side with 20% roundedness
  beforePath.quadTo(beforeRect.right(), centerY - trackThickness / 2, beforeRect.right(), centerY);
  beforePath.quadTo(
    beforeRect.right(), centerY + trackThickness / 2, beforeRect.right() - trackThickness * 0.2,
    centerY + trackThickness / 2);

  // Close the path to the left side
  beforePath.lineTo(beforeRect.left() + trackThickness * 0.5, centerY + trackThickness / 2);
  beforePath.quadTo(beforeRect.left(), centerY + trackThickness / 2, beforeRect.left(), centerY);

  painter.fillPath(beforePath, activeTrackColor);

  // After track path (asymmetric roundedness)
  QPainterPath afterPath;
  afterPath.moveTo(afterRect.left(), centerY + trackThickness / 2);
  afterPath.quadTo(
    afterRect.left(), centerY - trackThickness / 2, afterRect.left() + trackThickness * 0.2,
    centerY - trackThickness / 2);
  afterPath.lineTo(afterRect.right() - trackThickness * 0.5, centerY - trackThickness / 2);
  afterPath.quadTo(afterRect.right(), centerY - trackThickness / 2, afterRect.right(), centerY);
  afterPath.quadTo(
    afterRect.right(), centerY + trackThickness / 2, afterRect.right() - trackThickness * 0.5,
    centerY + trackThickness / 2);
  afterPath.lineTo(afterRect.left() + trackThickness * 0.2, centerY + trackThickness / 2);
  afterPath.quadTo(afterRect.left(), centerY + trackThickness / 2, afterRect.left(), centerY);
  painter.fillPath(afterPath, inactiveTrackColor);

  painter.setBrush(QColor("#8bd0f0"));
  int handleLineHeight = 25;
  int handleLineWidth = 4;
  int handleLineRadius = 2;
  QRect handleLineRect(
    handleRect.center().x() - handleLineWidth / 2, centerY - handleLineHeight / 2, handleLineWidth,
    handleLineHeight);
  QPainterPath handlePath;
  handlePath.addRoundedRect(handleLineRect, handleLineRadius, handleLineRadius);
  painter.fillPath(handlePath, QColor("#8bd0f0"));
}
