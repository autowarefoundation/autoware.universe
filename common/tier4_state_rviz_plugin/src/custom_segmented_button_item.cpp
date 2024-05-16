#include "custom_segmented_button_item.hpp"

CustomSegmentedButtonItem::CustomSegmentedButtonItem(const QString & text, QWidget * parent)
: QPushButton(text, parent),
  bgColor("#0F1417"),
  checkedBgColor("#354A54"),
  inactiveTextColor("#8a9297"),
  activeTextColor("#d0e6f2")
{
  setCheckable(true);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
}

void CustomSegmentedButtonItem::setColors(
  const QColor & bg, const QColor & checkedBg, const QColor & activeText,
  const QColor & inactiveText)
{
  bgColor = bg;
  checkedBgColor = checkedBg;
  activeTextColor = activeText;
  inactiveTextColor = inactiveText;
  update();
}

// void CustomSegmentedButtonItem::updateSize()
// {
//   QFontMetrics fm(font());
//   int width = fm.horizontalAdvance(text()) + 40;  // Add padding
//   int height = fm.height() + 20;                  // Add padding
//   setFixedSize(width, height);
// }

void CustomSegmentedButtonItem::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Determine the button's color based on its state
  QColor buttonColor;
  if (isHovered && !isChecked()) {
    buttonColor = hoverColor;
  } else {
    buttonColor = isChecked() ? checkedBgColor : bgColor;
  }
  // Determine if this is the first or last button
  bool isFirstButton = false;
  bool isLastButton = false;

  QHBoxLayout * parentLayout = qobject_cast<QHBoxLayout *>(parentWidget()->layout());
  if (parentLayout) {
    int index = parentLayout->indexOf(this);
    isFirstButton = (index == 0);
    isLastButton = (index == parentLayout->count() - 1);
  }

  // Draw button background

  QRect buttonRect = rect().adjusted(1, 1, -1, -1);  // Adjust to fill the space;

  //   make it shorter in height by making both top and bottom 1 less
  buttonRect.setTop(buttonRect.top() + 1);
  buttonRect.setBottom(buttonRect.bottom() - 1);
  QPainterPath path;
  int radius = (height() - 2) / 2;

  if (isFirstButton) {
    path.moveTo(buttonRect.right(), buttonRect.top());
    path.arcTo(buttonRect.left(), buttonRect.top() - 0.5, 2 * radius, 2 * radius, 90, 180);
    path.lineTo(buttonRect.right(), buttonRect.bottom());
    path.lineTo(buttonRect.right(), buttonRect.top());
  } else if (isLastButton) {
    path.moveTo(buttonRect.left(), buttonRect.top());
    path.arcTo(
      buttonRect.right() - 2 * radius, buttonRect.top() - 0.5, 2 * radius, 2 * radius, 90, -180);
    path.lineTo(buttonRect.left(), buttonRect.bottom());
    path.lineTo(buttonRect.left(), buttonRect.top());
  } else {
    path.addRect(buttonRect);
  }
  painter.fillPath(path, buttonColor);

  // Draw button text
  painter.setPen(isChecked() ? activeTextColor : inactiveTextColor);
  painter.drawText(rect(), Qt::AlignCenter, text());
}

void CustomSegmentedButtonItem::enterEvent(QEvent * event)
{
  isHovered = true;
  update();
  QPushButton::enterEvent(event);
}

void CustomSegmentedButtonItem::leaveEvent(QEvent * event)
{
  isHovered = false;
  update();
  QPushButton::leaveEvent(event);
}