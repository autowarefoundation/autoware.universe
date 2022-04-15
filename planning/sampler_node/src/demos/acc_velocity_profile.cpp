/*
 * Copyright 2022 Autoware Foundation. All rights reserved.
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

#include <qt5/QtCore/QCoreApplication>
#include <qt5/QtWidgets/QDoubleSpinBox>
#include <qt5/QtWidgets/QCheckBox>
#include <qt5/QtWidgets/QGroupBox>
#include <qt5/QtWidgets/QComboBox>
#include <frenet_planner/polynomials.hpp>

#include <qcustomplot.h>

class DemoWindow : public QMainWindow {
    public:
            QCustomPlot * plot = new QCustomPlot;
            QCPGraph * s_graph = plot->addGraph();
            QCPGraph * v_graph = plot->addGraph();
            QCPGraph * a_graph = plot->addGraph();
            QCPGraph * j_graph = plot->addGraph();
            QCPGraph * acc_s_graph = plot->addGraph();
            QCPGraph * acc_v_graph = plot->addGraph();
            QDoubleSpinBox * s0 = new QDoubleSpinBox;
            QDoubleSpinBox * sT = new QDoubleSpinBox;
            QDoubleSpinBox * v0 = new QDoubleSpinBox;
            QDoubleSpinBox * vT = new QDoubleSpinBox;
            QDoubleSpinBox * a0 = new QDoubleSpinBox;
            QDoubleSpinBox * aT = new QDoubleSpinBox;
            QDoubleSpinBox * duration = new QDoubleSpinBox;
            QDoubleSpinBox * acc_s0 = new QDoubleSpinBox;
            QDoubleSpinBox * acc_v0 = new QDoubleSpinBox;
            QDoubleSpinBox * acc_dist = new QDoubleSpinBox;
            QCheckBox * acc_check = new QCheckBox("ACC");
            QComboBox * graph_select = new QComboBox;


        DemoWindow() {
            setMinimumSize(200, 200);
            auto * vlayout = new QVBoxLayout;
            auto * poly_layout = new QHBoxLayout;
            auto * acc_layout = new QHBoxLayout;
            auto * central = new QGroupBox;
            central->setLayout(vlayout);
            auto * poly_params = new QGroupBox;
            auto * acc_params = new QGroupBox;
            poly_params->setLayout(poly_layout);
            acc_params->setLayout(acc_layout);
            setCentralWidget(central);
            plot->setSizePolicy(QSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding));
            vlayout->addWidget(plot);
            vlayout->addWidget(poly_params);
            vlayout->addWidget(acc_params);
            s0->setValue(0.0);
            sT->setValue(50.0);
            sT->setMaximum(1e12);
            v0->setValue(0.0);
            vT->setValue(5.0);
            a0->setValue(0.0);
            aT->setValue(0.0);
            duration->setValue(10.0);
            acc_s0->setValue(25.0);
            acc_v0->setValue(5.0);
            acc_dist->setValue(5.0);
            poly_layout->addWidget(new QLabel("s0"));
            poly_layout->addWidget(s0);
            poly_layout->addWidget(new QLabel("v0"));
            poly_layout->addWidget(v0);
            poly_layout->addWidget(new QLabel("a0"));
            poly_layout->addWidget(a0);
            poly_layout->addWidget(new QLabel("sT"));
            poly_layout->addWidget(sT);
            poly_layout->addWidget(new QLabel("vT"));
            poly_layout->addWidget(vT);
            poly_layout->addWidget(new QLabel("aT"));
            poly_layout->addWidget(aT);
            poly_layout->addWidget(new QLabel("T"));
            poly_layout->addWidget(duration);
            acc_layout->addWidget(new QLabel("Plot:"));
            graph_select->addItems({"position", "velocity", "acceleration", "jerk"});
            acc_layout->addWidget(graph_select);
            acc_check->setChecked(false);
            acc_layout->addWidget(acc_check);
            acc_layout->addWidget(new QLabel("Initial position"));
            acc_layout->addWidget(acc_s0);
            acc_layout->addWidget(new QLabel("Initial velocity"));
            acc_layout->addWidget(acc_v0);
            acc_layout->addWidget(new QLabel("Follow distance"));
            acc_layout->addWidget(acc_dist);

            s_graph->setPen(QPen(Qt::blue));
            v_graph->setPen(QPen(Qt::blue));
            a_graph->setPen(QPen(Qt::blue));
            j_graph->setPen(QPen(Qt::blue));
            acc_s_graph->setPen(QPen(Qt::red));
            acc_v_graph->setPen(QPen(Qt::red));

            updatePlot();

            connect(s0, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(v0, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(a0, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(sT, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(vT, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(aT, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(duration, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(acc_check, QOverload<int>::of(&QCheckBox::stateChanged), [&](int){updatePlot();});
            connect(acc_s0, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(acc_v0, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(acc_dist, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [&](double){updatePlot();});
            connect(graph_select, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int){updatePlot();});
        }

    public slots:
        void updatePlot() const {
            constexpr double resolution = 0.1;

            QVector<double> ss;
            QVector<double> ts;
            if(acc_check->isChecked()) {
                for(auto t = 0.0; t <= duration->value(); t += resolution) {
                    ss.push_back(acc_s0->value() - acc_dist->value() + t * acc_v0->value());
                    ts.push_back(t);
                }
                acc_s_graph->setData(ts, ss, true);
                acc_v_graph->setData(ts, QVector<double>(ts.size(), acc_v0->value()), true);

                sT->setValue(acc_s0->value() - acc_dist->value() + duration->value() * acc_v0->value());
                vT->setValue(acc_v0->value());
            }
            else {
                acc_s_graph->setData({}, {});
                acc_v_graph->setData({}, {});
            }

            ss.clear();
            ts.clear();
            QVector<double> vs;
            QVector<double> as;
            QVector<double> js;
            frenet_planner::Polynomial velocity_polynomial(s0->value(), v0->value(), a0->value(), sT->value(), vT->value(), aT->value(), duration->value());
            for(auto t = 0.0; t <= duration->value(); t += resolution) {
                ts.push_back(t);
                ss.push_back(velocity_polynomial.position(t));
                vs.push_back(velocity_polynomial.velocity(t));
                as.push_back(velocity_polynomial.acceleration(t));
                js.push_back(velocity_polynomial.jerk(t));
            }
            s_graph->setData(ts, ss, true);
            v_graph->setData(ts, vs, true);
            a_graph->setData(ts, as, true);
            j_graph->setData(ts, js, true);

            if(graph_select->currentText() != "position") {
                s_graph->setData({}, {});
                acc_s_graph->setData({}, {});
            }
            if(graph_select->currentText() != "velocity") {
                v_graph->setData({}, {});
                acc_v_graph->setData({}, {});
            }
            if(graph_select->currentText() != "acceleration") {
                a_graph->setData({}, {});
            }
            if(graph_select->currentText() != "jerk") {
                j_graph->setData({}, {});
            }

            plot->rescaleAxes();
            plot->replot();
        }

        void updateTargets() {
        }

};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    DemoWindow window;
    window.show();
    return QApplication::exec();
}