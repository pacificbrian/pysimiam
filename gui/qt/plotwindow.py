import sys
import numpy
from random import random

mplPlotWindow = None
qwtPlotWindow = None
pqgPlotWindow = None
PlotWindow = None

def use_qwt_backend():
    global PlotWindow, qwtPlotWindow
    if qwtPlotWindow is None:
        qwtPlotWindow = __import__('plotwindow_qwt',
                                   globals(), locals(),
                                   ['PlotWindow'], 1).PlotWindow
    PlotWindow = qwtPlotWindow

def use_qtgraph_backend():
    global PlotWindow, pqgPlotWindow
    if pqgPlotWindow is None:
        pqgPlotWindow = __import__('plotwindow_qtgraph',
                                   globals(), locals(),
                                   ['PlotWindow'], 1).PlotWindow
    PlotWindow = pqgPlotWindow

def use_matplotlib_backend():
    global PlotWindow, mplPlotWindow
    if mplPlotWindow is None:
        mplPlotWindow = __import__('plotwindow_mpl',
                                   globals(), locals(),
                                   ['PlotWindow'], 1).PlotWindow
    PlotWindow = mplPlotWindow    

def use_some_backend():
    global PlotWindow
    if PlotWindow is not None:
        return
    try:
        use_qtgraph_backend()
    except ImportError:
        try:
            use_qwt_backend()
        except ImportError:
            try:
                use_matplotlib_backend()
            except ImportError: 
                raise ImportError("No suitable plot backend found")
    if PlotWindow is None:
        raise ImportError("No suitable plot backend found")
        
def create_predefined_plot_window(plots):
    """Create a window with plots from plot dictionary"""
    try:
        use_some_backend()
    except ImportError as e:
        print(str(e))
        return None, None
    w = PlotWindow()
    es = []
    for plot in plots:
        p = w.add_plot()
        for l,e,c in plot:
            p.add_curve(l,e,c)        
            es.append(e)
    return es, w
