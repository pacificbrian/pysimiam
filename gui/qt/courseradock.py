from .Qt import QtGui, QtCore, Widgets

from core.helpers import Struct
from collections import OrderedDict
from traceback import format_exception
import sys
from core.coursera import CourseraException

class CourseraDock(Widgets.QDockWidget):
    
    closed = QtCore.Signal(bool)
    
    btn_default_stylesheet = """background-color: rgb(216, 229, 226);
                                border: 1px solid black;
                                text-align: left;
                                padding: 10px;"""
    btn_complete_stylesheet= """background-color: rgb(60, 255, 60);
                                border: 1px solid black;
                                text-align: left;
                                padding: 10px;"""
    btn_error_stylesheet   = """background-color: rgb(255, 60, 60);
                                border: 1px solid black;
                                text-align: left;
                                padding: 10px;"""
    
    def __init__(self, parent, tester):
        """Construct a new dockwindow following the tester """
        self.tester = tester
        
        Widgets.QDockWidget.__init__(self, tester.testname, parent)
        self.setAllowedAreas(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea | QtCore.Qt.DockWidgetArea.RightDockWidgetArea)
        
        if not self.widget():
            self.setWidget(Widgets.QWidget(self))
        
        vl = Widgets.QVBoxLayout(self.widget())
        #self.widget().setLayout(vl)
        
        panel = Widgets.QFrame(self)
        vl.addWidget(panel)
        
        self.login = Widgets.QLineEdit(self)
        self.login.textEdited.connect(self.check_logpass)
        self.password = Widgets.QLineEdit(self)
        self.password.setEchoMode(Widgets.QLineEdit.EchoMode.Password)
        self.password.textEdited.connect(self.check_logpass)
        
        self.cache = QtCore.QSettings('pySimiam','coursera')
        if sys.version_info[0] < 3:
            self.login.setText(self.cache.value('username','').toString())
            self.password.setText(self.cache.value('password','').toString())
        else:
            self.login.setText(self.cache.value('username',''))
            self.password.setText(self.cache.value('password',''))
        
        self.tests = []
        
        fl = Widgets.QFormLayout(panel)
        fl.addRow('&Login:',self.login)
        fl.addRow('&Password:',self.password)
        panel.setLayout(fl)
        
        panel = Widgets.QFrame(self)
        panel.setFrameShadow(Widgets.QFrame.Shadow.Sunken)
        panel.setFrameShape(Widgets.QFrame.Shape.Panel)
        vl.addWidget(panel)
        
        vl2 = Widgets.QVBoxLayout(panel)
        
        signalmapper = QtCore.QSignalMapper(self)
        signalmapper.mapped[int].connect(self.test)
        
        for i,test in enumerate(self.tester.tests):
            btn = Widgets.QPushButton("Test {}: {}".format(i+1,test.name),panel)
            btn.setStyleSheet(self.btn_default_stylesheet)
            btn.setEnabled(False)
            vl2.addWidget(btn)
            self.tests.append(btn)
            signalmapper.setMapping(btn,i)
            btn.clicked.connect(signalmapper.map)
        
        panel.setLayout(vl2)
        
        self.text = Widgets.QLabel("Enter your Coursera login and assignments password and push one of the test buttons above to run the test and submit the results to Coursera.")
        self.text.setWordWrap(True)
        vl.addWidget(self.text)
        self.text.setFrameShadow(Widgets.QFrame.Shadow.Sunken)
        self.text.setFrameShape(Widgets.QFrame.Shape.Panel)
        self.text.setMargin(5)

        #vl.setStretch(2,1)
        vl.addStretch(1)
        
        self.check_logpass() # Enable if login/password match
    
    def enable_testing(self,enable):
        for btn in self.tests:
            btn.setEnabled(enable)
    
    @QtCore.Slot(int)
    def test(self,i):
        self.enable_testing(False)
        self.tester.setuser(self.login.text(),self.password.text())
        try:
            self.tester.test(i, lambda r: self.feedback(i,r))
        except CourseraException as e:
            self.text.setText(str(e))
        
    def feedback(self,index,message):
        if message == "Fantastic!":
            self.tests[index].setStyleSheet(self.btn_complete_stylesheet)
        else:
            self.tests[index].setStyleSheet(self.btn_error_stylesheet)
        
        self.text.setText(message)
        self.enable_testing(True)
        
    def check_logpass(self):
        if sys.version_info[0] < 3:
            valid_ = not self.login.text().isEmpty() and not self.password.text().isEmpty()
        else:
            valid_ = bool(self.login.text()) and bool(self.password.text())
        if valid_:
            self.cache.setValue('username',self.login.text())
            self.cache.setValue('password',self.password.text())
        self.enable_testing(valid_)
        
    def closeEvent(self,event):
        super(CourseraDock,self).closeEvent(event)
        if event.isAccepted():
            print('closed')
            self.closed.emit(True)        
            
