#ifndef CUSTOM_CONTAINER_HPP
#define CUSTOM_CONTAINER_HPP

#include <QFrame>
#include <QGridLayout>
#include <QPainter>
#include <QPainterPath>

class CustomContainer : public QFrame
{
  Q_OBJECT

public:
  explicit CustomContainer(QWidget * parent = nullptr);

  // Methods to set dimensions and corner radius
  void setContainerHeight(int height);
  void setContainerWidth(int width);
  void setCornerRadius(int radius);

  // Getters
  int getContainerHeight() const;
  int getContainerWidth() const;
  int getCornerRadius() const;
  QGridLayout * getLayout() const;  // Add a method to access the layout

protected:
  void paintEvent(QPaintEvent * event) override;
  QSize sizeHint() const override;
  QSize minimumSizeHint() const override;

private:
  QGridLayout * layout;
  int cornerRadius;
};

#endif  // CUSTOM_CONTAINER_HPP
