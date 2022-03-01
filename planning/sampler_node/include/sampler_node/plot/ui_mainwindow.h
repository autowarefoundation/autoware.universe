/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include <qcustomplot.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
  QWidget * centralWidget;
  QVBoxLayout * verticalLayout;
  QFrame * frame_2;
  QVBoxLayout * verticalLayout_3;
  QCustomPlot * customPlot;
  QFrame * frame;
  QVBoxLayout * verticalLayout_2;
  QLabel * label;
  QMenuBar * menuBar;
  QStatusBar * statusBar;

  void setupUi(QMainWindow * MainWindow)
  {
    if (MainWindow->objectName().isEmpty())
      MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
    MainWindow->resize(621, 515);
    centralWidget = new QWidget(MainWindow);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    verticalLayout = new QVBoxLayout(centralWidget);
    verticalLayout->setSpacing(6);
    verticalLayout->setContentsMargins(11, 11, 11, 11);
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    frame_2 = new QFrame(centralWidget);
    frame_2->setObjectName(QString::fromUtf8("frame_2"));
    frame_2->setFrameShape(QFrame::StyledPanel);
    frame_2->setFrameShadow(QFrame::Sunken);
    frame_2->setLineWidth(1);
    frame_2->setMidLineWidth(0);
    verticalLayout_3 = new QVBoxLayout(frame_2);
    verticalLayout_3->setSpacing(0);
    verticalLayout_3->setContentsMargins(0, 0, 0, 0);
    verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
    customPlot = new QCustomPlot(frame_2);
    customPlot->setObjectName(QString::fromUtf8("customPlot"));
    QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(customPlot->sizePolicy().hasHeightForWidth());
    customPlot->setSizePolicy(sizePolicy);

    verticalLayout_3->addWidget(customPlot);

    verticalLayout->addWidget(frame_2);

    frame = new QFrame(centralWidget);
    frame->setObjectName(QString::fromUtf8("frame"));
    frame->setFrameShape(QFrame::StyledPanel);
    frame->setFrameShadow(QFrame::Raised);
    verticalLayout_2 = new QVBoxLayout(frame);
    verticalLayout_2->setSpacing(6);
    verticalLayout_2->setContentsMargins(11, 11, 11, 11);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    label = new QLabel(frame);
    label->setObjectName(QString::fromUtf8("label"));

    verticalLayout_2->addWidget(label);

    verticalLayout->addWidget(frame);

    MainWindow->setCentralWidget(centralWidget);
    menuBar = new QMenuBar(MainWindow);
    menuBar->setObjectName(QString::fromUtf8("menuBar"));
    menuBar->setGeometry(QRect(0, 0, 621, 25));
    MainWindow->setMenuBar(menuBar);
    statusBar = new QStatusBar(MainWindow);
    statusBar->setObjectName(QString::fromUtf8("statusBar"));
    MainWindow->setStatusBar(statusBar);

    retranslateUi(MainWindow);

    QMetaObject::connectSlotsByName(MainWindow);
  }  // setupUi

  void retranslateUi(QMainWindow * MainWindow)
  {
    MainWindow->setWindowTitle(
      QApplication::translate("MainWindow", "QCustomPlot Interaction Example", nullptr));
    label->setText(QApplication::translate(
      "MainWindow",
      "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" "
      "\"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
      "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
      "p, li { white-space: pre-wrap; }\n"
      "</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; "
      "font-style:normal;\">\n"
      "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; "
      "-qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Select the "
      "axes</span> to drag and zoom them individually.</p>\n"
      "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; "
      "-qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Double click "
      "labels</span> or legend items to set user specified strings.</p>\n"
      "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; "
      "-qt-block-indent:0; text-indent:0px;\"><span style=\" font-weight:600;\">Left click</span> "
      "on graphs or legend to select graphs.</p>\n"
      "<p style=\" m"
      "argin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; "
      "text-indent:0px;\"><span style=\" font-weight:600;\">Right click</span> for a popup menu to "
      "add/remove graphs and move the legend</p></body></html>",
      nullptr));
  }  // retranslateUi
};

namespace Ui
{
class MainWindow : public Ui_MainWindow
{
};
}  // namespace Ui

QT_END_NAMESPACE

#endif  // UI_MAINWINDOW_H
