#ifndef CUSTOM_BUTTON_HPP_
#define CUSTOM_BUTTON_HPP_

#include <QColor>
#include <QFontMetrics>
#include <QGraphicsDropShadowEffect>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>

class CustomElevatedButton : public QPushButton
{
  Q_OBJECT

public:
  explicit CustomElevatedButton(
    const QString & text, const QColor & bgColor = QColor("#171C1F"),
    const QColor & textColor = QColor("#8bd0f0"), QWidget * parent = nullptr);
  void updateStyle(
    const QString & text, const QColor & bgColor, const QColor & textColor,
    const QColor & hoverColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  void enterEvent(QEvent * event) override;
  void leaveEvent(QEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QColor backgroundColor = QColor("##171C1F");
  QColor textColor = QColor("#8bd0f0");
  QColor hoverColor = QColor("#3C3F41");
  bool isHovered = false;
};

#endif  // CUSTOM_BUTTON_HPP_
