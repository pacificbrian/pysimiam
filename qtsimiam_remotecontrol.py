#!/usr/bin/env python
# QtSimiam for Coursera Week 7
# Author: Tim Fuchs <typograph@elec.ru>
# Description: This is the top-level application for QtSimiam.
import sys

from gui.qt.Qt import QApplication
from gui.qt.remote import SimulationWidget
from core.coursera import Week7

if __name__ == "__main__":
    app = QApplication(sys.argv)
    simWidget = SimulationWidget()
    simWidget.show()
    app.exec()
