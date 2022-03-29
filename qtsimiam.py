#!/usr/bin/env python
# QtSimiam main executable
# Author: Tim Fuchs <typograph@elec.ru>
# Description: This is the top-level application for QtSimiam.
import sys

from gui.qt.Qt import QApplication
from gui.qt.mainwindow import SimulationWidget

if __name__ == "__main__":
    app = QApplication(sys.argv)
    simWidget = SimulationWidget()
    simWidget.show()
    if len(sys.argv) > 1:
        if len(sys.argv) == 2:
            simWidget.load_world(sys.argv[1])
        else:
            print("Too many command-line options")
    app.exec()
