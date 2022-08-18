/*
 * Copyright 2022 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <eigen3/unsupported/Eigen/Splines>
#include <qt5/QtCore/QCoreApplication>
#include <qt5/QtWidgets/QCheckBox>
#include <qt5/QtWidgets/QComboBox>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QGroupBox>

#include <qcustomplot.h>

#include <iostream>

class DemoWindow : public QMainWindow
{
public:
  QCustomPlot * plot = new QCustomPlot;
  QCPGraph * curve = plot->addGraph();
  QCPGraph * points = plot->addGraph();

  DemoWindow()
  {
    setMinimumSize(200, 200);
    auto * vlayout = new QVBoxLayout;
    auto * central = new QGroupBox;
    central->setLayout(vlayout);
    setCentralWidget(central);
    plot->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));
    vlayout->addWidget(plot);

    curve->setPen(QPen(Qt::blue));
    curve->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));
    points->setPen(QPen(Qt::red));
    points->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 4));

    Eigen::RowVectorXd knots(33);
    knots << 0, 0, 0, 0.037037, 0.0740741, 0.111111, 0.148148, 0.185185, 0.222222, 0.259259,
      0.296296, 0.333333, 0.37037, 0.407407, 0.444444, 0.481481, 0.518519, 0.555556, 0.592593,
      0.62963, 0.666667, 0.703704, 0.740741, 0.777778, 0.814815, 0.851852, 0.888889, 0.925926,
      0.962963, 1, 1, 1, 1;

    Eigen::MatrixXd ctrls(27, 2);
    ctrls << 3714.3, 73691.6, 3713.85, 73692.5, 3713.77, 73692.6, 3713.39, 73693.4, 3712.93,
      73694.3, 3712.47, 73695.1, 3712.02, 73696, 3711.57, 73696.9, 3711.56, 73696.9, 3711.1,
      73697.8, 3710.64, 73698.7, 3710.18, 73699.6, 3709.73, 73700.5, 3709.37, 73701.2, 3709.26,
      73701.4, 3708.79, 73702.2, 3708.32, 73703.1, 3707.86, 73704, 3707.39, 73704.9, 3706.94,
      73705.8, 3706.5, 73706.7, 3706.07, 73707.6, 3705.65, 73708.5, 3705.25, 73709.4, 3705.17,
      73709.6, 3704.86, 73710.3, 3704.79, 73710.5;

    QVector<double> xs;
    QVector<double> ys;
    for (auto i = 0; i < ctrls.rows(); ++i) {
      xs.push_back(ctrls(i, 0));
      ys.push_back(ctrls(i, 1));
    }
    points->setData(xs, ys);

    ctrls.transposeInPlace();
    const auto spline = Eigen::Spline<double, 2, 3>(knots, ctrls);

    xs.clear();
    ys.clear();
    for (auto knot = 0.00; knot <= 1.00; knot += 0.01) {
      const auto p = spline(knot);
      std::cout << knot << " " << p.transpose() << std::endl;
      if (!xs.empty() && (std::abs(p.x() - xs.back()) > 10 || std::abs(p.y() - ys.back()) > 10)) {
        std::cout << "\tSKIP\n";
        continue;
      }
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
    curve->setData(xs, ys);

    updatePlot();
  }

public slots:
  void updatePlot() const
  {
    plot->rescaleAxes();
    plot->replot();
  }
};

int main(int argc, char * argv[])
{
  QApplication app(argc, argv);
  DemoWindow window;
  window.show();
  return QApplication::exec();
}
