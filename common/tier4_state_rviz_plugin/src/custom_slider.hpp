#ifndef CUSTOM_SLIDER_HPP
#define CUSTOM_SLIDER_HPP

#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QSlider>
#include <QStyleOptionSlider>

class CustomSlider : public QSlider
{
  Q_OBJECT

public:
  explicit CustomSlider(Qt::Orientation orientation, QWidget * parent = nullptr);
  QColor green = QColor("#00e678");

protected:
  void paintEvent(QPaintEvent * event) override;
};

#endif  // CUSTOM_SLIDER_HPP
