#!/usr/bin/env python
# QtSimiam for Coursera Week 7
# Author: Tim Fuchs <typograph@elec.ru>
# Description: This is the top-level application for QtSimiam.
import sys

from gui.qt.Qt import QtGui
from gui.qt.mainwindow import SimulationWidget
from core.coursera import Week7

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    simWidget = SimulationWidget()
    simWidget.superv_action.trigger()
    simWidget.show()
    simWidget.setTestSuite(Week7)
    simWidget.load_world("week7.xml")
    app.exec()
