#ifndef CUSTOM_TOGGLE_SWITCH_HPP_
#define CUSTOM_TOGGLE_SWITCH_HPP_

#include <QCheckBox>
#include <QColor>
#include <QMouseEvent>
#include <QPainter>
class CustomToggleSwitch : public QCheckBox
{
  Q_OBJECT

public:
  explicit CustomToggleSwitch(QWidget * parent = nullptr);
  QSize sizeHint() const override;
  void setCheckedState(bool state);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  bool blockSignalsGuard = false;  // Guard variable to block signals during updates
};

#endif  // CUSTOM_TOGGLE_SWITCH_HPP_
