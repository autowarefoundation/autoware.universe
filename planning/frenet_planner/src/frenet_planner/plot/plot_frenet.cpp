/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
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

#include "frenet_planner/plot/debug_window.hpp"

#include <QApplication>

#include <chrono>
#include <vector>

int main(int argc, char * argv[])
{
  QApplication a(argc, argv);
  frenet_planner::plot::MainWindow w;
  w.show();
  return a.exec();
}
