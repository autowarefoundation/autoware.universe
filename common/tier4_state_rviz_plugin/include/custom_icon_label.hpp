#ifndef CUSTOM_ICON_LABEL_HPP_
#define CUSTOM_ICON_LABEL_HPP_

#include <QColor>
#include <QLabel>
#include <QMap>
#include <QPainter>
#include <QPainterPath>
#include <QPixmap>
#include <QStyleOption>
#include <QWidget>

enum IconState { Active, Pending, Danger, None, Crash };

class CustomIconLabel : public QLabel
{
  Q_OBJECT

public:
  explicit CustomIconLabel(const QColor & bgColor = QColor("#2C3E50"), QWidget * parent = nullptr);
  void updateStyle(IconState state, const QColor & bgColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  void loadIcons();
  QPixmap icon;
  QColor backgroundColor;
  QMap<IconState, QPixmap> iconMap;
};

#endif  // CUSTOM_ICON_LABEL_HPP_
