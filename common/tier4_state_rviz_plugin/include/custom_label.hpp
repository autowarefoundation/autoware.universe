#ifndef CUSTOMLABEL_HPP
#define CUSTOMLABEL_HPP

#include <QLabel>
#include <QWidget>

#include <qcolor.h>

class CustomLabel : public QLabel
{
  Q_OBJECT

public:
  explicit CustomLabel(const QString & text, QWidget * parent = nullptr);
  void updateStyle(const QString & text, const QColor & bgColor, const QColor & textColor);

protected:
  void paintEvent(QPaintEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QColor backgroundColor = QColor("#171C1F");
  QColor textColor = QColor("#FFFFFF");
};

#endif  // CUSTOMLABEL_HPP
