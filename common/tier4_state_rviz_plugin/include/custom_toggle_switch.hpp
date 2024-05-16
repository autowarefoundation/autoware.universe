// CustomToggleSwitch.h
#ifndef CUSTOMTOGGLESWITCH_H
#define CUSTOMTOGGLESWITCH_H

#include <QCheckBox>
#include <QColor>
#include <QMouseEvent>
#include <QPainter>

class CustomToggleSwitch : public QCheckBox
{
  Q_OBJECT

public:
  explicit CustomToggleSwitch(QWidget * parent = nullptr);
  QSize sizeHint() const override;  // Declare the sizeHint method

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;

private:
  bool isDragging = false;
  QPoint dragStartPoint;
};

#endif  // CUSTOMTOGGLESWITCH_H
