#include "include/custom_container.hpp"

CustomContainer::CustomContainer(QWidget * parent) : QFrame(parent), cornerRadius(15)
{
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
  setContentsMargins(0, 0, 0, 0);
  layout = new QGridLayout(this);
  layout->setMargin(0);                      // Margin between the border and child widgets
  layout->setSpacing(0);                     // Spacing between child widgets
  layout->setContentsMargins(10, 0, 10, 0);  // Margin between the border and the layout
  setLayout(layout);
}

void CustomContainer::setCornerRadius(int radius)
{
  cornerRadius = radius;
  update();
}

int CustomContainer::getCornerRadius() const
{
  return cornerRadius;
}

QGridLayout * CustomContainer::getLayout() const
{
  return layout;  // Provide access to the layout
}

QSize CustomContainer::sizeHint() const
{
  QSize size = layout->sizeHint();
  int width = size.width() + 20;    // Adding padding
  int height = size.height() + 20;  // Adding padding
  return QSize(width, height);
}

QSize CustomContainer::minimumSizeHint() const
{
  return sizeHint();
}

void CustomContainer::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw background
  QPainterPath path;
  path.addRoundedRect(rect(), height() / 2, height() / 2);  // Use height for rounded corners
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#0f1417"));  // Background color
  painter.drawPath(path);
}
