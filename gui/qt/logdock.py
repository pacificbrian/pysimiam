from .Qt import QtGui, QtCore, Widgets

class LogDock(Widgets.QDockWidget):
    closed = QtCore.Signal(bool)
    def __init__(self, parent):
        """Construct a new dockwindow with the log """
        
        Widgets.QDockWidget.__init__(self, "Message log", parent)
        self.setAllowedAreas(QtCore.Qt.DockWidgetArea.TopDockWidgetArea | QtCore.Qt.DockWidgetArea.BottomDockWidgetArea)

        self.table = Widgets.QTableWidget(0,2,self)
        self.table.setHorizontalHeaderLabels(["Sender","Message"])

        try:
            self.table.verticalHeader().setResizeMode(Widgets.QHeaderView.ResizeToContents)
        except AttributeError: # Qt6
            self.table.verticalHeader().setSectionResizeMode(Widgets.QHeaderView.ResizeMode.ResizeToContents)

        hhdrs = self.table.horizontalHeader()       
        try:
            hhdrs.setResizeMode(0, Widgets.QHeaderView.ResizeToContents)
            hhdrs.setResizeMode(1, Widgets.QHeaderView.Stretch)
        except TypeError: # Qt5
            hhdrs.setSectionResizeMode(0, Widgets.QHeaderView.ResizeToContents)
            hhdrs.setSectionResizeMode(1, Widgets.QHeaderView.Stretch)
        except AttributeError: # Qt6
            hhdrs.setSectionResizeMode(0, Widgets.QHeaderView.ResizeMode.ResizeToContents)
            hhdrs.setSectionResizeMode(1, Widgets.QHeaderView.ResizeMode.Stretch)

        self.setWidget(self.table)

    def append(self,message,name,color):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row,0,Widgets.QTableWidgetItem(name))
        self.table.setItem(row,1,Widgets.QTableWidgetItem(message))
        clr = Widgets.QTableWidgetItem(" ")
        self.table.setVerticalHeaderItem(row,clr)
        if color is not None:
            clr.setBackground(QtGui.QColor(color))

    def closeEvent(self,event):
        super(LogDock,self).closeEvent(event)
        if event.isAccepted():
            print('closed')
            self.closed.emit(True)
