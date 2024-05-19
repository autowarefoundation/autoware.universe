#include "include/custom_segmented_button.hpp"

#include <qsizepolicy.h>

CustomSegmentedButton::CustomSegmentedButton(QWidget * parent)
: QWidget(parent), buttonGroup(new QButtonGroup(this)), layout(new QHBoxLayout(this))
{
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  layout->setContentsMargins(0, 0, 0, 0);  // Ensure no margins around the layout
  layout->setSpacing(0);                   // Ensure no spacing between buttons

  setLayout(layout);

  buttonGroup->setExclusive(true);

  connect(
    buttonGroup, QOverload<int>::of(&QButtonGroup::idClicked), this,
    &CustomSegmentedButton::buttonClicked);
}

CustomSegmentedButtonItem * CustomSegmentedButton::addButton(const QString & text)
{
  CustomSegmentedButtonItem * button = new CustomSegmentedButtonItem(text);
  layout->addWidget(button);
  buttonGroup->addButton(button, layout->count() - 1);

  return button;
}

QButtonGroup * CustomSegmentedButton::getButtonGroup() const
{
  return buttonGroup;
}

QSize CustomSegmentedButton::sizeHint() const
{
  return QSize(400, 40);  // Adjust the size hint as needed

  // return QSize(
  //   layout->count() * (layout->itemAt(0)->widget()->width()),
  //   layout->itemAt(0)->widget()->height() + 10);
}

QSize CustomSegmentedButton::minimumSizeHint() const
{
  return sizeHint();
}

void CustomSegmentedButton::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw background
  QPainterPath path;
  path.addRoundedRect(rect(), height() / 2, height() / 2);

  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor("#0F1417"));
  painter.drawPath(path);
}

void CustomSegmentedButton::drawBorders(QPainter & painter)
{
  painter.setPen(QPen(QColor("#8a9297"), 3));  // Color for the borders
  const QList<QAbstractButton *> buttons = buttonGroup->buttons();
  for (int i = 1; i < buttons.size(); ++i) {
    QAbstractButton * button = buttons[i];
    QRect buttonRect = button->geometry();
    int x = buttonRect.left();
    painter.drawLine(x, 0, x, height());
  }
}
