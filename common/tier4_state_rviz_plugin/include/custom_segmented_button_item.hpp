#ifndef CUSTOM_SEGMENTED_BUTTON_ITEM_HPP_
#define CUSTOM_SEGMENTED_BUTTON_ITEM_HPP_

#include <QColor>
#include <QHBoxLayout>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QStyleOption>

class CustomSegmentedButtonItem : public QPushButton
{
  Q_OBJECT

public:
  explicit CustomSegmentedButtonItem(const QString & text, QWidget * parent = nullptr);

  void setColors(
    const QColor & bg, const QColor & checkedBg, const QColor & activeText,
    const QColor & inactiveText);
  void setActivated(bool activated);
  void setCheckableButton(bool checkable);
  void setDisabledButton(bool disabled);
  void setHovered(bool hovered);

protected:
  void paintEvent(QPaintEvent * event) override;
  void enterEvent(QEvent * event) override;
  void leaveEvent(QEvent * event) override;

private:
  void updateCheckableState();

  QColor bgColor;
  QColor checkedBgColor;
  QColor hoverColor = QColor("#3C3F41");
  QColor inactiveTextColor;
  QColor activeTextColor;
  bool isHovered = false;
  bool isActivated = false;
  bool isDisabled = false;
};

#endif  // CUSTOM_SEGMENTED_BUTTON_ITEM_HPP_
