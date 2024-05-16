#include "custom_label.hpp"

#include <QColor>
#include <QGraphicsDropShadowEffect>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QStyleOption>

CustomLabel::CustomLabel(const QString & text, QWidget * parent) : QLabel(text, parent)
{
  setFont(QFont("Roboto", 10, QFont::Bold));  // Set the font as needed
  setAlignment(Qt::AlignCenter);

  // Add shadow effect
  QGraphicsDropShadowEffect * shadowEffect = new QGraphicsDropShadowEffect(this);
  shadowEffect->setBlurRadius(15);               // Blur radius for a smoother shadow
  shadowEffect->setOffset(3, 3);                 // Offset for the shadow
  shadowEffect->setColor(QColor(0, 0, 0, 120));  // Shadow color with transparency
  setGraphicsEffect(shadowEffect);
}

QSize CustomLabel::sizeHint() const
{
  QFontMetrics fm(font());
  int textWidth = fm.horizontalAdvance(text());
  int textHeight = fm.height();
  int width = textWidth + 40;  // Adding padding
  int height = textHeight;     // Adding padding
  return QSize(width, height);
}

QSize CustomLabel::minimumSizeHint() const
{
  return sizeHint();
}

void CustomLabel::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  int cornerRadius = height() / 2;  // Making the corner radius proportional to the height

  // Draw background
  QPainterPath path;
  path.addRoundedRect(rect().adjusted(1, 1, -1, -1), cornerRadius, cornerRadius);

  painter.setPen(Qt::NoPen);
  painter.setBrush(backgroundColor);

  painter.drawPath(path);

  // Set text color and draw text
  painter.setPen(textColor);
  painter.drawText(rect(), Qt::AlignCenter, text());
}

void CustomLabel::updateStyle(
  const QString & text, const QColor & bg_color, const QColor & text_color)
{
  setText(text);
  backgroundColor = bg_color;
  textColor = text_color;
  update();  // Force repaint
}
