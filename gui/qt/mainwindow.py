#!/usr/bin/python
#QtSimiam
#Author: Tim Fuchs
#Description: This is the top-level application for QtSimiam.
import sys
from .Qt import QtGui, QtCore, Widgets
import os
from traceback import format_exception

from core.ui import SimUI
from core.simulator import Simulator

from .renderer import QtRenderer
from .paramwindow import ParamDock, DockManager
from .courseradock import CourseraDock
from .logdock import LogDock
from .plotwindow import create_predefined_plot_window # ,create_plot_window

class PlayPauseAction(QtGui.QAction):
    def __init__(self, parent, run_slot, pause_slot):
        QtGui.QAction.__init__(self, parent)
        self.playset = (QtGui.QIcon.fromTheme("media-playback-start",
                            QtGui.QIcon("./res/image/arrow-right.png")),
                        "Run",
                        run_slot,
                        "Run simulation")
        self.pauseset = (QtGui.QIcon.fromTheme("media-playback-pause",
                            QtGui.QIcon("./res/image/media-playback-pause-7.png")),
                         "Pause",
                         pause_slot,
                         "Pause simulation")
        self.triggered.connect(self.__on_click)
        self.reset()
        
    def __on_click(self):
        self.callback()
        self.set_state()

    def reset(self):
        self.click_to_run = False
        self.set_state(self.playset)
        
    def set_state(self, actset = None):
        if actset is None:
            if self.click_to_run:
                actset = self.pauseset
            else:
                actset = self.playset
        self.click_to_run = not self.click_to_run
        self.setIcon(actset[0])
        self.setText(actset[1])
        self.callback = actset[2]
        self.setStatusTip(actset[3])

class SimulationWidget(SimUI, Widgets.QMainWindow):
    def __init__(self,parent=None):
        Widgets.QMainWindow.__init__(self,parent)
        self.setWindowTitle("QtSimiam")
        self.setWindowIcon(QtGui.QIcon("./res/image/appicon.png"))
        self.resize(1250,1000)
        
        self.create_actions()
        self.create_toolbars()
        self.create_menu()
        self.create_statusbar()
        # Set intro message
        self.status_label.setText("Welcome to QtSimiam")
        
        # create XML file dialog
        self.world_dialog = Widgets.QFileDialog(self,
                                "Select World File",
                                "worlds", 
                                "WorldFile (*.xml)")
        self.world_dialog.setAcceptMode(Widgets.QFileDialog.AcceptMode.AcceptOpen)
        self.world_dialog.setFileMode(Widgets.QFileDialog.FileMode.ExistingFile)

        # create supervisor file dialog
        
        if sys.version_info.major == 3:
            formats = [bytes(fmt).decode('utf-8') for fmt in QtGui.QImageWriter.supportedImageFormats()]
        else:
            formats = [str(fmt) for fmt in QtGui.QImageWriter.supportedImageFormats()]
        
        fmtstring = "All supported image formats ({});;{}".format( \
                    " ".join('*.' + fmt for fmt in formats), \
                    ";;".join( "{} files (*.{})".format(fmt.upper(), fmt) for fmt in formats ))
        
        self.screenshot_dialog = Widgets.QFileDialog(self, "Export view", ".", fmtstring)
        self.screenshot_dialog.setAcceptMode(Widgets.QFileDialog.AcceptMode.AcceptSave)
        self.screenshot_dialog.setFileMode(Widgets.QFileDialog.FileMode.AnyFile)
       
        scrollArea = Widgets.QScrollArea(self)
        self.setCentralWidget(scrollArea)
        self.viewer = SimulatorViewer()
        self.viewer.resized.connect(self.refresh_view)
        scrollArea.setWidget(self.viewer)
        scrollArea.setWidgetResizable(True)
        
        self.__screenshot_filename = ""
        self.__clear_graph_on_start = False
        self.plots = []

        self.setDockOptions(self.dockOptions() | Widgets.QMainWindow.DockOption.AllowNestedDocks)
        
        self.coursera_dock = None
        self.logdock = None
        self.add_logdock()

        self.dockmanager = DockManager(self)
        self.dockmanager.apply_request.connect(self.apply_parameters)

        self.sim_timer = QtCore.QTimer(self)
        self.sim_timer.setInterval(10)
        self.sim_timer.timeout.connect(self.update_time)
        
        SimUI.__init__(self,self.viewer.renderer, Simulator)

        self.sim_timer.start()

    def create_actions(self):
        
        self.open_world_action = \
            QtGui.QAction(QtGui.QIcon.fromTheme("document-open",
                            QtGui.QIcon("./res/image/open.png")),
                          "Open XML &World",
                          self)
        self.open_world_action.triggered.connect(self.on_open_world)
        self.open_world_action.setShortcut(QtGui.QKeySequence(QtGui.QKeySequence.StandardKey.Open))

        self.open_world_action.setStatusTip("Open a new simulation")

        self.screenshot_action = \
            QtGui.QAction(QtGui.QIcon.fromTheme("camera-photo",
                            QtGui.QIcon("./res/image/screenshot.png")),
                          "Export &screenshot",
                          self)
        self.screenshot_action.triggered.connect(self.on_screenshot)
        self.screenshot_action.setStatusTip("Export current simulation view")
                            
        self.exit_action = \
            QtGui.QAction(QtGui.QIcon.fromTheme("application-exit"),
                    "E&xit",
                    self)
        self.exit_action.triggered.connect(self.close)
        self.exit_action.setShortcut(QtGui.QKeySequence(QtGui.QKeySequence.StandardKey.Quit))
        self.exit_action.setToolTip("Quit the Program")
        self.exit_action.setStatusTip("Exit QtSimiam")
        
        self.rev_action = \
            QtGui.QAction(QtGui.QIcon.fromTheme("media-seek-backward",
                            QtGui.QIcon("./res/image/arrow-left-double.png")),
                          "Rewind", self)
        self.rev_action.triggered.connect(self.on_rewind)
        self.rev_action.setStatusTip("Reset simulation")
        
        self.run_action = PlayPauseAction(self, self.on_run,self.on_pause)        
        self.run_action.setEnabled(False)
        
        self.step_action = \
            QtGui.QAction(QtGui.QIcon.fromTheme("media-skip-forward",
                            QtGui.QIcon("./res/image/media-skip-forward-7.png")),
                          "Step", self)
        self.step_action.triggered.connect(self.on_step)
        self.step_action.setStatusTip("Do one simulation step")
        self.step_action.setEnabled(False)
        
        self.grid_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/grid.png"),
                          "Show/Hide grid", self)
        self.grid_action.setStatusTip("Show/hide grid")
        self.grid_action.triggered[bool].connect(self.show_grid)
        self.grid_action.setCheckable(True)
        self.grid_action.setChecked(False)

        self.sens_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/robot-sensors.png"),
                          "Show/Hide sensors", self)
        self.sens_action.setStatusTip("Show/hide robot sensors")
        self.sens_action.triggered[bool].connect(self.show_sensors)
        self.sens_action.setCheckable(True)
        self.sens_action.setChecked(True)

        self.trace_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/robot-tracks.png"),
                          "Show/Hide robot trajectores", self)
        self.trace_action.setStatusTip("Show/hide robot trajectores")
        self.trace_action.triggered[bool].connect(self.show_tracks)
        self.trace_action.setCheckable(True)
        self.trace_action.setChecked(True)

        self.superv_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/robot-supervisors.png"),
                          "Show/Hide supervisor information", self)
        self.superv_action.setStatusTip("Show/hide supervisor information")
        self.superv_action.triggered[bool].connect(self.show_supervisors)
        self.superv_action.setCheckable(True)
        self.superv_action.setChecked(False)

        zoom_group = QtGui.QActionGroup(self)

        self.zoom_world_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/zoom-scene.png"),
                          "Show all", self)
        self.zoom_world_action.triggered.connect(self.zoom_scene)
        self.zoom_world_action.setStatusTip("Show the whole world in view")
        self.zoom_world_action.setCheckable(True)
        self.zoom_world_action.setChecked(True)
        zoom_group.addAction(self.zoom_world_action)
        
        self.zoom_robot_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/zoom-robot.png"),
                          "Follow robot", self)
        self.zoom_robot_action.triggered.connect(self.zoom_robot)
        self.zoom_robot_action.setStatusTip("Center the view on robot")
        self.zoom_robot_action.setCheckable(True)
        self.zoom_robot_action.setChecked(False)
        zoom_group.addAction(self.zoom_robot_action)

        self.rotate_action = \
            QtGui.QAction(QtGui.QIcon("./res/image/zoom-robot-rot.png"),
                          "Follow robot orientation", self)
        self.rotate_action.triggered.connect(self.rot_robot)
        self.rotate_action.setStatusTip("Rotate the view with the robot")
        self.rotate_action.setCheckable(True)
        self.rotate_action.setChecked(False)
        self.rotate_action.setEnabled(False)

        self.showlog_action = QtGui.QAction("Show log",self)
        self.showlog_action.triggered.connect(self.add_logdock)
        
        self.about_action = \
            QtGui.QAction(QtGui.QIcon.fromTheme("help-about",
                            self.windowIcon()),
                          "About",self)
        self.about_action.setStatusTip("About QtSimiam")
        self.about_action.triggered.connect(self.about)
        
    def create_toolbars(self):
        
        self.simulator_toolbar = Widgets.QToolBar("Control",self)
        self.simulator_toolbar.setAllowedAreas(QtCore.Qt.ToolBarArea.TopToolBarArea | QtCore.Qt.ToolBarArea.BottomToolBarArea)
        
        self.simulator_toolbar.addAction(self.open_world_action)
        self.simulator_toolbar.addAction(self.screenshot_action)
        self.simulator_toolbar.addSeparator()
        
        self.simulator_toolbar.addAction(self.rev_action)
        self.simulator_toolbar.addAction(self.run_action)
        self.simulator_toolbar.addAction(self.step_action)
        
        self.speed_slider = Widgets.QSlider(QtCore.Qt.Orientation.Horizontal,self)
        self.speed_slider.setToolTip("Adjust speed")
        self.speed_slider.setStatusTip("Adjust simulation speed")
        self.speed_slider.setTickPosition(Widgets.QSlider.TickPosition.NoTicks)
        self.speed_slider.setMaximumWidth(300)
        self.speed_slider.setRange(-100,100)
        self.speed_slider.setValue(0)
        self.speed_slider.setEnabled(False)
        self.speed_slider.valueChanged[int].connect(self.scale_time)
        self.simulator_toolbar.addWidget(self.speed_slider)
        
        self.speed_label = Widgets.QLabel(" Speed: 1.0x ",self)
        self.speed_label.setToolTip("Current speed multiplier")
        self.simulator_toolbar.addWidget(self.speed_label)
                       
        self.addToolBar(self.simulator_toolbar)

        self.view_toolbar = Widgets.QToolBar("View",self)
        self.view_toolbar.setAllowedAreas(QtCore.Qt.ToolBarArea.TopToolBarArea | QtCore.Qt.ToolBarArea.BottomToolBarArea)

        self.view_toolbar.addAction(self.grid_action)        
        self.view_toolbar.addAction(self.sens_action)
        self.view_toolbar.addAction(self.trace_action)
        self.view_toolbar.addAction(self.superv_action)
        self.view_toolbar.addSeparator()
        
        self.view_toolbar.addAction(self.zoom_world_action)
        self.view_toolbar.addAction(self.zoom_robot_action)
        self.view_toolbar.addAction(self.rotate_action)
        
        self.zoom_slider = Widgets.QSlider(QtCore.Qt.Orientation.Horizontal,self)
        self.zoom_slider.setTickPosition(Widgets.QSlider.TickPosition.NoTicks)
        self.zoom_slider.setToolTip("Adjust zoom")
        self.zoom_slider.setStatusTip("Zoom in/out on robot")
        self.zoom_slider.setMaximumWidth(180)
        self.zoom_slider.setRange(-140,80)
        self.zoom_slider.setValue(-40)
        self.zoom_slider.setEnabled(False)
        self.zoom_slider.valueChanged[int].connect(self.scale_zoom)
        self.view_toolbar.addWidget(self.zoom_slider)
        self.zoom_label = Widgets.QLabel(" Zoom: 1.0x ",self)
        self.zoom_label.setToolTip("Current zoom factor")
        self.view_toolbar.addWidget(self.zoom_label)
        
        self.zoom_factor = 0
                       
        self.addToolBar(self.view_toolbar)

    def create_menu(self):
        menu = Widgets.QMenuBar(self)
        self.setMenuBar(menu)
        
        file_menu = menu.addMenu("&File")
        
        file_menu.addAction(self.open_world_action)
        file_menu.addAction(self.screenshot_action)
        file_menu.addSeparator()
        file_menu.addAction(self.exit_action)
        
        view_menu = menu.addMenu("&View")
        
        view_menu.addAction(self.zoom_world_action)
        view_menu.addAction(self.zoom_robot_action)
        view_menu.addAction(self.rotate_action)
        view_menu.addSeparator()
        
        view_menu.addAction(self.grid_action)
        view_menu.addAction(self.sens_action)
        view_menu.addAction(self.trace_action)
        view_menu.addAction(self.superv_action)
        
        run_menu = menu.addMenu("&Simulation")
        
        run_menu.addAction(self.run_action)
        run_menu.addAction(self.step_action)
        run_menu.addAction(self.rev_action)
        
        self.run_menu = run_menu
        
        help_menu = menu.addMenu("&Help")
        help_menu.addAction(self.showlog_action)
        help_menu.addSeparator()
        help_menu.addAction(self.about_action)
        
    def create_statusbar(self):      
        self.setStatusBar(Widgets.QStatusBar())
        self.status_label = Widgets.QLabel("",self.statusBar())
        self.status_label.setFrameShape(Widgets.QFrame.Shape.NoFrame)
        self.statusBar().addWidget(self.status_label)

    def closeEvent(self,event):
        self.sim_timer.stop()
        self.run_simulator_command('stop')
        while self.simulator_thread.is_alive():
            self.process_events(True)
            self.simulator_thread.join(0.1)
        while self.plots:
            self.plots.pop().close()            
        super(SimulationWidget,self).closeEvent(event)

    def load_world(self,filename):
        self.run_action.setEnabled(False)
        if not os.path.exists(filename):
            filename = os.path.join('worlds',filename)
            if not os.path.exists(filename):
                print("Cannot open file {}".format(filename))
                return
        self.dockmanager.clear()
        self.run_simulator_command('read_config',filename)

    def setTestSuite(self,TestClass):
        self.menuBar().addSeparator()
        self.coursera_action = self.menuBar().addAction("&Submit assignment")
        
        tb = Widgets.QToolButton(self)
        tb.setDefaultAction(self.coursera_action)
        self.coursera_action.setIcon(QtGui.QIcon("./res/image/coursera.png"))
        self.simulator_toolbar.insertWidget(self.rev_action, tb)
        self.simulator_toolbar.insertSeparator(self.rev_action)
        
        self.coursera_action.triggered.connect(self.create_coursera_widget)

        self.tester = TestClass(self)
        # Don't auto show coursera login (use above button)
        #self.create_coursera_widget()
     
    def create_coursera_widget(self):
        self.coursera_action.setEnabled(False)
        if self.coursera_dock is None:
            self.coursera_dock = CourseraDock(self, self.tester)
            self.addDockWidget(QtCore.Qt.DockWidgetArea.LeftDockWidgetArea, self.coursera_dock)
            self.coursera_dock.closed.connect(self.coursera_action.setEnabled)
        else:
            self.coursera_dock.show()

    def add_logdock(self):
        self.showlog_action.setEnabled(False)
        if self.logdock is None:
            self.logdock = LogDock(self)
            self.addDockWidget(QtCore.Qt.DockWidgetArea.BottomDockWidgetArea,self.logdock)
            self.logdock.closed.connect(self.showlog_action.setEnabled)
        else:
            self.logdock.show()
        
    # Slots
    def about(self):
        Widgets.QMessageBox.about(self,"About QtSimiam",
        """<b>PySimiam (Qt)</b><br>
        Robot simulator<br>
        &copy; Pysimiam Team
        """)
    
    @QtCore.Slot()
    def on_rewind(self): # Start from the beginning
        self.speed_slider.setEnabled(False)
        #self.time_label.setText("00:00.0")
        self.run_simulator_command('reset_simulation')

    @QtCore.Slot()
    def on_run(self): # Run/unpause
        self.run_simulation()

    @QtCore.Slot()
    def on_pause(self): # Pause
        self.speed_slider.setEnabled(False)        
        self.pause_simulation()

    @QtCore.Slot()
    def on_step(self): # Pause
        #self.speed_slider.setEnabled(False)
        self.step_simulation()

    @QtCore.Slot()
    def on_open_world(self):
        self.on_pause()
        if self.world_dialog.exec():
            self.load_world(self.world_dialog.selectedFiles()[0])

    @QtCore.Slot()
    def on_screenshot(self):
        if self.screenshot_dialog.exec():
            # Remember, direct access to the renderer is not thread-safe
            if self.simulator_thread.is_running():
                self.__screenshot_filename = self.screenshot_dialog.selectedFiles()[0]
            else:
                self.viewer.export_bitmap(self.screenshot_dialog.selectedFiles()[0])

    @QtCore.Slot()
    def refresh_view(self):
        self.run_simulator_command('refresh')
            
    @QtCore.Slot(bool)
    def show_grid(self,show):
        self.run_simulator_command('show_grid',show)

    @QtCore.Slot(bool)
    def show_sensors(self,show):
        self.run_simulator_command('show_sensors',show)
            
    @QtCore.Slot(bool)
    def show_tracks(self,show):
        self.run_simulator_command('show_tracks',show)
            
    @QtCore.Slot(bool)
    def show_supervisors(self,show):
        self.run_simulator_command('show_supervisors',show)
            
    @QtCore.Slot()
    def zoom_scene(self):
        self.zoom_slider.setEnabled(False)
        self.rotate_action.setEnabled(False)
        self.run_simulator_command('focus_on_world')

    @QtCore.Slot()
    def zoom_robot(self):
        self.zoom_slider.setEnabled(True)
        self.rotate_action.setEnabled(True)
        self.run_simulator_command('focus_on_robot',self.rotate_action.isChecked())
        self.run_simulator_command('adjust_zoom',5.0**(self.zoom_slider.value()/100.0))

    @QtCore.Slot()
    def rot_robot(self):
        self.run_simulator_command('focus_on_robot',self.rotate_action.isChecked())
            
    @QtCore.Slot(int)
    def scale_zoom(self,value):
        zoom = 5.0**(value/100.0)
        self.run_simulator_command('adjust_zoom',zoom)
        self.zoom_label.setText(" Zoom: %.1fx "%(zoom))

    @QtCore.Slot(int)
    def scale_time(self,value):
        m = 10.0**((value-self.zoom_factor)/100.0)
        self.run_simulator_command('set_time_multiplier',m)
        self.speed_label.setText(" Speed: %.1fx "%m)

    @QtCore.Slot()
    def update_time(self):
        if self.simulator_thread.is_running():
            t = self.simulator_thread.get_time()
            minutes = int(t//60)
            #self.time_label.setText("%02d:%04.1f"%(minutes,t - minutes*60))
            self.status_label.setText(
                "Simulation running... {:02d}:{:04.1f}".format(minutes,t - minutes*60))
        if not self.screenshot_dialog.isVisible(): # During screenshot the simulator is paused
            self.process_events(True)
        
    def apply_parameters(self, robot_id, params):
        self.run_simulator_command('apply_parameters', robot_id, params)
        
    def start_testing(self):
        self.open_world_action.setEnabled(False)
        self.run_menu.setEnabled(False)
        self.simulator_toolbar.setEnabled(False)

    def stop_testing(self):
        self.open_world_action.setEnabled(True)
        self.run_menu.setEnabled(True)
        self.simulator_toolbar.setEnabled(True)

    def _new_plot(self,exprs, plot):
        if exprs:
            plot.show()
            self.plots.append(plot)
            for expr in exprs:
                self.run_simulator_command('add_plotable',expr)        
                
    def add_graph(self, description):
        self._new_plot(*create_predefined_plot_window(description))
            
### Simulator events

    def simulator_make_param_window(self,robot_id,name,parameters):       
        # FIXME adding to the right for no reason
        self.dockmanager.add_dock_right(robot_id, name, parameters)
        
    def simulator_running(self):
        self.speed_slider.setEnabled(True)
        self.step_action.setEnabled(False)
        if self.__clear_graph_on_start:
            self.__clear_graph_on_start = False
            for plot in self.plots:
                plot.clear_data()
    
    def simulator_paused(self):
        self.speed_slider.setEnabled(False)
        self.step_action.setEnabled(True)
        t = self.simulator_thread.get_time()
        minutes = int(t//60)
        self.status_label.setText(
            "Simulation paused... {:02d}:{:04.1f}".format(minutes,t - minutes*60))

    def simulator_reset(self):
        self.run_action.reset()
        self.run_action.setEnabled(True)
        self.status_label.setText("Simulation ready")
        self.__clear_graph_on_start = True
 
    def simulator_stopped(self):
        # FIXME this function isn't necessary
        self.speed_slider.setEnabled(False)
        
    def simulator_update_view(self):
        self.viewer.update_bitmap()
        if len(self.__screenshot_filename):
            self.viewer.export_bitmap(self.__screenshot_filename)
            self.__screenshot_filename = ""
        
    def simulator_exception(self,e_type, e_value, e_traceback):
        Widgets.QMessageBox.critical(self,"{}: {}".format(e_type.__name__,e_value),"\n".join(format_exception(e_type,e_value,e_traceback)))
        self.run_action.setEnabled(False)
        
    def simulator_log(self, message, objclass, objcolor):
        self.logdock.append(message,objclass,objcolor)
        
    def simulator_plot_update(self,data):
        for plot in self.plots:
            plot.add_data(data)        
            
#end QtSimiamFrame class

class SimulatorViewer(Widgets.QFrame):
    
    resized = QtCore.Signal()
    
    def __init__(self, parent = None):
        super(SimulatorViewer, self).__init__(parent)
        self.bitmap = QtGui.QPixmap()
        self.blt_bitmap = QtGui.QImage(self.size(), QtGui.QImage.Format.Format_ARGB32)
        self.renderer = QtRenderer(self.blt_bitmap)
        self.resize_on_paint = False

    def paintEvent(self, event):
        super(SimulatorViewer, self).paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(),QtCore.Qt.GlobalColor.white)
        s = self.bitmap.rect().size()
        s.scale(self.rect().size(),QtCore.Qt.AspectRatioMode.KeepAspectRatio)
        dx = (self.width() - s.width())/2
        dy = (self.height() - s.height())/2
        painter.drawPixmap(QtCore.QRect(QtCore.QPoint(dx,dy),s),self.bitmap,self.bitmap.rect())

    def export_bitmap(self,filename):
        """Saves the view into a file."""
        self.bitmap.save(filename)
        
    def update_bitmap(self):
        self.bitmap = QtGui.QPixmap.fromImage(self.blt_bitmap)
        # resize the canvas - at this point nothing is being drawn
        if self.resize_on_paint:
            self.blt_bitmap = QtGui.QImage(self.width(),
                                            self.height(),
                                            QtGui.QImage.Format.Format_ARGB32)
            self.renderer.set_canvas(self.blt_bitmap)          
            self.resize_on_paint = False
        self.update()

    def resizeEvent(self,event):
        """Resize panel and canvas"""
        # use cached size and flag
        self.resize_on_paint = True
        self.resized.emit()
