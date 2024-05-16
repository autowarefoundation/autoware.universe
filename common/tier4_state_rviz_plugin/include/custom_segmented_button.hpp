#ifndef CUSTOMSEGMENTEDBUTTON_HPP
#define CUSTOMSEGMENTEDBUTTON_HPP

#include "custom_segmented_button_item.hpp"

#include <QButtonGroup>
#include <QColor>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>
#include <QWidget>

class CustomSegmentedButton : public QWidget
{
  Q_OBJECT

public:
  explicit CustomSegmentedButton(QWidget * parent = nullptr);

  CustomSegmentedButtonItem * addButton(const QString & text);
  QButtonGroup * getButtonGroup() const;

  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

Q_SIGNALS:
  void buttonClicked(int id);

protected:
  void paintEvent(QPaintEvent * event) override;

private:
  void drawBorders(QPainter & painter);  // Method to draw borders between buttons

  QButtonGroup * buttonGroup;
  QHBoxLayout * layout;
};

#endif  // CUSTOMSEGMENTEDBUTTON_HPP
