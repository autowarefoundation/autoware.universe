#include "custom_button.hpp"

CustomElevatedButton::CustomElevatedButton(
  const QString & text, const QColor & bgColor, const QColor & textColor, QWidget * parent)
: QPushButton(text, parent), backgroundColor(bgColor), textColor(textColor)
{
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  setCursor(Qt::PointingHandCursor);

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
  int width = textWidth + 40;    // Adding padding
  int height = textHeight + 20;  // Adding padding
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