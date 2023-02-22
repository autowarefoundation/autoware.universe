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

#include "qboxlayout.h"

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
  QVBoxLayout * candidates_tab_layout;
  QWidget * candidates_tab_widget;
  QVBoxLayout * candidates_tab_widget_layout;
  QSplitter * candidates_splitter;
  QTableWidget * candidates_table;
  QWidget * candidates_widget;
  QVBoxLayout * cand_layout;
  QCustomPlot * cand_pos;
  QCustomPlot * cand_vel;
  QCustomPlot * cand_acc;
  QCustomPlot * cand_jerk;
  QWidget * reusable_tab;
  QVBoxLayout * reusable_tab_layout;
  QWidget * reusable_tab_widget;
  QVBoxLayout * reusable_tab_widget_layout;
  QSplitter * reusable_splitter;
  QTableWidget * reusable_table;
  QWidget * reusable_widget;
  QVBoxLayout * reuse_layout;
  QCustomPlot * reuse_pos;
  QCustomPlot * reuse_vel;
  QCustomPlot * reuse_acc;
  QCustomPlot * reuse_jerk;
  QWidget * frenet_tab;
  QVBoxLayout * frenet_tab_layout;
  QCustomPlot * frenet_s;
  QCustomPlot * frenet_d;
  QWidget * bezier_tab;
  QWidget * pruning_tab;
  QCustomPlot * pruning_tab_plot;
  QVBoxLayout * pruning_tab_layout;
  QWidget * obstacles_tab;
  QCustomPlot * obstacles_tab_plot;
  QVBoxLayout * obstacles_tab_layout;
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

    // Candidates tab
    candidates_tab = new QWidget();
    candidates_tab->setObjectName(QString::fromUtf8("candidates_tab"));
    candidates_tab_layout = new QVBoxLayout(candidates_tab);
    candidates_tab_layout->setObjectName(QString::fromUtf8("candidates_tab_layout"));
    candidates_tab_widget = new QWidget(candidates_tab);
    candidates_tab_widget->setObjectName(QString::fromUtf8("candidates_tab_widget"));
    candidates_tab_widget_layout = new QVBoxLayout(candidates_tab_widget);
    candidates_tab_widget_layout->setObjectName(QString::fromUtf8("candidates_tab_widget_layout"));
    candidates_splitter = new QSplitter(candidates_tab_widget);
    candidates_splitter->setObjectName(QString::fromUtf8("candidates_splitter"));
    candidates_splitter->setOrientation(Qt::Horizontal);
    candidates_table = new QTableWidget(candidates_splitter);
    candidates_table->setObjectName(QString::fromUtf8("candidates_table"));
    candidates_splitter->addWidget(candidates_table);
    candidates_widget = new QWidget(candidates_splitter);
    candidates_widget->setObjectName(QString::fromUtf8("candidates_widget"));
    cand_layout = new QVBoxLayout(candidates_widget);
    cand_layout->setObjectName(QString::fromUtf8("cand_layout"));
    cand_pos = new QCustomPlot(candidates_widget);
    cand_pos->setObjectName(QString::fromUtf8("cand_pos"));

    cand_layout->addWidget(cand_pos);

    cand_vel = new QCustomPlot(candidates_widget);
    cand_vel->setObjectName(QString::fromUtf8("cand_vel"));

    cand_layout->addWidget(cand_vel);

    cand_acc = new QCustomPlot(candidates_widget);
    cand_acc->setObjectName(QString::fromUtf8("cand_acc"));

    cand_layout->addWidget(cand_acc);

    cand_jerk = new QCustomPlot(candidates_widget);
    cand_jerk->setObjectName(QString::fromUtf8("cand_jerk"));

    cand_layout->addWidget(cand_jerk);

    candidates_splitter->addWidget(candidates_widget);

    candidates_tab_widget_layout->addWidget(candidates_splitter);

    candidates_tab_layout->addWidget(candidates_tab_widget);
    tabWidget->addTab(candidates_tab, QString());

    // Reusables tab
    reusable_tab = new QWidget();
    reusable_tab->setObjectName(QString::fromUtf8("reusable_tab"));
    reusable_tab_layout = new QVBoxLayout(reusable_tab);
    reusable_tab_layout->setObjectName(QString::fromUtf8("reusable_tab_layout"));
    reusable_tab_widget = new QWidget(reusable_tab);
    reusable_tab_widget->setObjectName(QString::fromUtf8("reusable_tab_widget"));
    reusable_tab_widget_layout = new QVBoxLayout(reusable_tab_widget);
    reusable_tab_widget_layout->setObjectName(QString::fromUtf8("reusable_tab_widget_layout"));
    reusable_splitter = new QSplitter(reusable_tab_widget);
    reusable_splitter->setObjectName(QString::fromUtf8("reusable_splitter"));
    reusable_splitter->setOrientation(Qt::Horizontal);
    reusable_table = new QTableWidget(reusable_splitter);
    reusable_table->setObjectName(QString::fromUtf8("reusable_table"));
    reusable_splitter->addWidget(reusable_table);
    reusable_widget = new QWidget(reusable_splitter);
    reusable_widget->setObjectName(QString::fromUtf8("reusable_widget"));
    reuse_layout = new QVBoxLayout(reusable_widget);
    reuse_layout->setObjectName(QString::fromUtf8("reuse_layout"));
    reuse_pos = new QCustomPlot(reusable_widget);
    reuse_pos->setObjectName(QString::fromUtf8("reuse_pos"));

    reuse_layout->addWidget(reuse_pos);

    reuse_vel = new QCustomPlot(reusable_widget);
    reuse_vel->setObjectName(QString::fromUtf8("reuse_vel"));

    reuse_layout->addWidget(reuse_vel);

    reuse_acc = new QCustomPlot(reusable_widget);
    reuse_acc->setObjectName(QString::fromUtf8("reuse_acc"));

    reuse_layout->addWidget(reuse_acc);

    reuse_jerk = new QCustomPlot(reusable_widget);
    reuse_jerk->setObjectName(QString::fromUtf8("reuse_jerk"));

    reuse_layout->addWidget(reuse_jerk);

    reusable_splitter->addWidget(reusable_widget);

    reusable_tab_widget_layout->addWidget(reusable_splitter);

    reusable_tab_layout->addWidget(reusable_tab_widget);
    tabWidget->addTab(reusable_tab, QString());

    // Frenet tab
    frenet_tab = new QWidget();
    frenet_tab->setObjectName(QString::fromUtf8("frenet_tab"));
    frenet_tab_layout = new QVBoxLayout(frenet_tab);
    frenet_tab_layout->setObjectName(QString::fromUtf8("frenet_tab_layout"));
    frenet_s = new QCustomPlot(frenet_tab);
    frenet_s->setObjectName(QString::fromUtf8("frenet_s"));

    frenet_tab_layout->addWidget(frenet_s);

    frenet_d = new QCustomPlot(frenet_tab);
    frenet_d->setObjectName(QString::fromUtf8("frenet_d"));

    frenet_tab_layout->addWidget(frenet_d);

    tabWidget->addTab(frenet_tab, QString());

    // Bézier tab
    bezier_tab = new QWidget();
    bezier_tab->setObjectName(QString::fromUtf8("bezier_tab"));
    tabWidget->addTab(bezier_tab, QString());
    pruning_tab = new QWidget();
    pruning_tab->setObjectName(QString::fromUtf8("pruning_tab"));
    tabWidget->addTab(pruning_tab, QString());
    pruning_tab_layout = new QVBoxLayout(pruning_tab);
    pruning_tab_plot = new QCustomPlot(pruning_tab);
    pruning_tab_layout->addWidget(pruning_tab_plot);

    // Obstacle tab
    obstacles_tab = new QWidget();
    obstacles_tab->setObjectName(QString::fromUtf8("obstacles_tab"));
    tabWidget->addTab(obstacles_tab, QString());
    obstacles_tab_layout = new QVBoxLayout(obstacles_tab);
    obstacles_tab_plot = new QCustomPlot(obstacles_tab);
    obstacles_tab_layout->addWidget(obstacles_tab_plot);

    // Selection tab
    selection_tab = new QWidget();
    selection_tab->setObjectName(QString::fromUtf8("selection_tab"));
    tabWidget->addTab(selection_tab, QString());

    // Performance tab
    perf_tab = new QWidget();
    perf_tab->setObjectName(QString::fromUtf8("perf_tab"));
    tabWidget->addTab(perf_tab, QString());

    verticalLayout->addWidget(tabWidget);

    verticalLayout_2->addLayout(verticalLayout);

    MainWindow->setCentralWidget(centralwidget);
    // Menu and status bars
    menubar = new QMenuBar(MainWindow);
    menubar->setObjectName(QString::fromUtf8("menubar"));
    menubar->setGeometry(QRect(0, 0, 801, 21));
    MainWindow->setMenuBar(menubar);
    statusbar = new QStatusBar(MainWindow);
    statusbar->setObjectName(QString::fromUtf8("statusbar"));
    MainWindow->setStatusBar(statusbar);

    retranslateUi(MainWindow);

    tabWidget->setCurrentIndex(0);

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
      tabWidget->indexOf(reusable_tab),
      QApplication::translate("MainWindow", "Reusable Trajectories", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(frenet_tab),
      QApplication::translate("MainWindow", "Frenet Sampler", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(bezier_tab),
      QApplication::translate("MainWindow", "Bezier Sampler", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(pruning_tab), QApplication::translate("MainWindow", "Pruning", nullptr));
    tabWidget->setTabText(
      tabWidget->indexOf(obstacles_tab),
      QApplication::translate("MainWindow", "Obstacles", nullptr));
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
