#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from PyQt5 import QtCore
from PyQt5.QtWidgets import QGridLayout
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QSizePolicy
from PyQt5.QtWidgets import QWidget


class TimeManagerWidget(QMainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUI()

    def setupUI(self):
        self.setObjectName("MainWindow")
        self.resize(480, 120)
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)

        self.centralwidget = QWidget(self)
        self.centralwidget.setObjectName("centralwidget")

        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setContentsMargins(10, 10, 10, 10)
        self.gridLayout.setObjectName("gridLayout")

        self.rate_button = []
        for i, rate in enumerate([0.1, 0.5, 1.0, 2.0, 5.0, 10.0]):
            btn = QPushButton(str(rate))
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            self.rate_button.append(btn)
            self.gridLayout.addWidget(self.rate_button[-1], 0, i, 1, 1)

        self.button = QPushButton("pause")
        self.button.setCheckable(True)
        self.button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.gridLayout.addWidget(self.button, 1, 0, 1, -1)
        self.setCentralWidget(self.centralwidget)

        """
    def onSetRate(self, button):
        print(button.text(), button)
        """
