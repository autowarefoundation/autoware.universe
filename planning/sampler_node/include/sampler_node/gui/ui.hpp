// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SAMPLER_NODE__GUI__UI_HPP_
#define SAMPLER_NODE__GUI__UI_HPP_

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include <qcustomplot.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
  QWidget * centralwidget;
  QVBoxLayout * verticalLayout_2;
  QVBoxLayout * verticalLayout;
  QTabWidget * tabWidget;
  QWidget * selected_tab;
  QGridLayout * selected_tab_layout;
  QCustomPlot * output_pos;
  QCustomPlot * output_vel;
  QCustomPlot * output_acc;
  QCustomPlot * output_jerk;
  QWidget * input_tab;
  QGridLayout * gridLayout;
  QCustomPlot * input_path;
  QCustomPlot * input_velocity;
  QCustomPlot * input_curvature;
  QLabel * input_occ_grid;
  QWidget * candidates_tab;
  QVBoxLayout * verticalLayout_9;
  QWidget * candidates_tab_widget;
  QVBoxLayout * verticalLayout_10;
  QSplitter * splitter;
  QTableWidget * candidates_table;
  QWidget * widget;
  QVBoxLayout * verticalLayout_11;
  QCustomPlot * cand_pos;
  QCustomPlot * cand_vel;
  QCustomPlot * cand_acc;
  QCustomPlot * cand_jerk;
  QWidget * frenet_tab;
  QVBoxLayout * verticalLayout_8;
  QCustomPlot * frenet_s;
  QCustomPlot * frenet_d;
  QWidget * bezier_tab;
  QWidget * pruning_tab;
  QWidget * selection_tab;
  QWidget * perf_tab;
  QMenuBar * menubar;
  QStatusBar * statusbar;

  void setupUi(QMainWindow * MainWindow)
  {
    if (MainWindow->objectName().isEmpty())
      MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
    MainWindow->resize(801, 613);
    centralwidget = new QWidget(MainWindow);
    centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
    verticalLayout_2 = new QVBoxLayout(centralwidget);
    verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
    verticalLayout = new QVBoxLayout();
    verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
    tabWidget = new QTabWidget(centralwidget);
    tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    selected_tab = new QWidget();
    selected_tab->setObjectName(QString::fromUtf8("selected_tab"));
    selected_tab_layout = new QGridLayout(selected_tab);
    selected_tab_layout->setObjectName(QString::fromUtf8("selected_tab_layout"));
    output_pos = new QCustomPlot(selected_tab);
    output_pos->setObjectName(QString::fromUtf8("output_pos"));

    selected_tab_layout->addWidget(output_pos, 0, 0, 1, 1);

    output_vel = new QCustomPlot(selected_tab);
    output_vel->setObjectName(QString::fromUtf8("output_vel"));
    output_vel->setLayoutDirection(Qt::LeftToRight);

    selected_tab_layout->addWidget(output_vel, 0, 1, 1, 1);

    output_acc = new QCustomPlot(selected_tab);
    output_acc->setObjectName(QString::fromUtf8("output_acc"));

    selected_tab_layout->addWidget(output_acc, 1, 0, 1, 1);

    output_jerk = new QCustomPlot(selected_tab);
    output_jerk->setObjectName(QString::fromUtf8("output_jerk"));

    selected_tab_layout->addWidget(output_jerk, 1, 1, 1, 1);

    tabWidget->addTab(selected_tab, QString());
    input_tab = new QWidget();
    input_tab->setObjectName(QString::fromUtf8("input_tab"));
    gridLayout = new QGridLayout(input_tab);
    gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
    input_path = new QCustomPlot(input_tab);
    input_path->setObjectName(QString::fromUtf8("input_path"));

    gridLayout->addWidget(input_path, 0, 0, 1, 1);

    input_velocity = new QCustomPlot(input_tab);
    input_velocity->setObjectName(QString::fromUtf8("input_velocity"));

    gridLayout->addWidget(input_velocity, 0, 1, 1, 1);

    input_curvature = new QCustomPlot(input_tab);
    input_curvature->setObjectName(QString::fromUtf8("input_curvature"));

    gridLayout->addWidget(input_curvature, 1, 0, 1, 1);

    input_occ_grid = new QLabel(input_tab);
    input_occ_grid->setObjectName(QString::fromUtf8("input_occ_grid"));
    input_occ_grid->setScaledContents(true);

    gridLayout->addWidget(input_occ_grid, 1, 1, 1, 1);

    tabWidget->addTab(input_tab, QString());
    candidates_tab = new QWidget();
    candidates_tab->setObjectName(QString::fromUtf8("candidates_tab"));
    verticalLayout_9 = new QVBoxLayout(candidates_tab);
    verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
    candidates_tab_widget = new QWidget(candidates_tab);
    candidates_tab_widget->setObjectName(QString::fromUtf8("candidates_tab_widget"));
    verticalLayout_10 = new QVBoxLayout(candidates_tab_widget);
    verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
    splitter = new QSplitter(candidates_tab_widget);
    splitter->setObjectName(QString::fromUtf8("splitter"));
    splitter->setOrientation(Qt::Horizontal);
    candidates_table = new QTableWidget(splitter);
    candidates_table->setObjectName(QString::fromUtf8("candidates_table"));
    splitter->addWidget(candidates_table);
    widget = new QWidget(splitter);
    widget->setObjectName(QString::fromUtf8("widget"));
    verticalLayout_11 = new QVBoxLayout(widget);
    verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
    cand_pos = new QCustomPlot(widget);
    cand_pos->setObjectName(QString::fromUtf8("cand_pos"));

    verticalLayout_11->addWidget(cand_pos);

    cand_vel = new QCustomPlot(widget);
    cand_vel->setObjectName(QString::fromUtf8("cand_vel"));

    verticalLayout_11->addWidget(cand_vel);

    cand_acc = new QCustomPlot(widget);
    cand_acc->setObjectName(QString::fromUtf8("cand_acc"));

    verticalLayout_11->addWidget(cand_acc);

    cand_jerk = new QCustomPlot(widget);
    cand_jerk->setObjectName(QString::fromUtf8("cand_jerk"));

    verticalLayout_11->addWidget(cand_jerk);

    splitter->addWidget(widget);

    verticalLayout_10->addWidget(splitter);

    verticalLayout_9->addWidget(candidates_tab_widget);

    tabWidget->addTab(candidates_tab, QString());
    frenet_tab = new QWidget();
    frenet_tab->setObjectName(QString::fromUtf8("frenet_tab"));
    verticalLayout_8 = new QVBoxLayout(frenet_tab);
    verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
    frenet_s = new QCustomPlot(frenet_tab);
    frenet_s->setObjectName(QString::fromUtf8("frenet_s"));

    verticalLayout_8->addWidget(frenet_s);

    frenet_d = new QCustomPlot(frenet_tab);
    frenet_d->setObjectName(QString::fromUtf8("frenet_d"));

    verticalLayout_8->addWidget(frenet_d);

    tabWidget->addTab(frenet_tab, QString());
    bezier_tab = new QWidget();
    bezier_tab->setObjectName(QString::fromUtf8("bezier_tab"));
    tabWidget->addTab(bezier_tab, QString());
    pruning_tab = new QWidget();
    pruning_tab->setObjectName(QString::fromUtf8("pruning_tab"));
    tabWidget->addTab(pruning_tab, QString());
    selection_tab = new QWidget();
    selection_tab->setObjectName(QString::fromUtf8("selection_tab"));
    tabWidget->addTab(selection_tab, QString());
    perf_tab = new QWidget();
    perf_tab->setObjectName(QString::fromUtf8("perf_tab"));
    tabWidget->addTab(perf_tab, QString());

    verticalLayout->addWidget(tabWidget);

    verticalLayout_2->addLayout(verticalLayout);

    MainWindow->setCentralWidget(centralwidget);
    menubar = new QMenuBar(MainWindow);
    menubar->setObjectName(QString::fromUtf8("menubar"));
    menubar->setGeometry(QRect(0, 0, 801, 21));
    MainWindow->setMenuBar(menubar);
    statusbar = new QStatusBar(MainWindow);
    statusbar->setObjectName(QString::fromUtf8("statusbar"));
    MainWindow->setStatusBar(statusbar);

    retranslateUi(MainWindow);

    tabWidget->setCurrentIndex(2);

    QMetaObject::connectSlotsByName(MainWindow);
  }  // setupUi

  void retranslateUi(QMainWindow * MainWindow) const
  {
    MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(selected_tab), QApplication::translate("MainWindow", "Selected", nullptr));
    input_occ_grid->setText(QString());
    tabWidget->setTabText(
      tabWidget->indexOf(input_tab), QApplication::translate("MainWindow", "Inputs", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(candidates_tab),
      QApplication::translate("MainWindow", "All Candidates", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(frenet_tab),
      QApplication::translate("MainWindow", "Frenet Sampler", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(bezier_tab),
      QApplication::translate("MainWindow", "Bezier Sampler", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(pruning_tab), QApplication::translate("MainWindow", "Pruning", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(selection_tab),
      QApplication::translate("MainWindow", "Selection", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(perf_tab), QApplication::translate("MainWindow", "Performances", nullptr));
  }  // retranslateUi
};

namespace Ui
{
class MainWindow : public Ui_MainWindow
{
};
}  // namespace Ui

QT_END_NAMESPACE

#endif  // SAMPLER_NODE__GUI__UI_HPP_
