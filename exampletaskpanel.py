from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ikplanner
from ddapp.ikparameters import IkParameters
from ddapp import planplayback
from ddapp import vtkNumpy as vnp

from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import os
import sys
import vtkAll as vtk
import numpy as np
import drc as lcmdrc

from PythonQt import QtCore, QtGui


class ContinuousWalkingTaskPlanner(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, planner):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.planner = planner

    def fit(self, polyData, points):
        pass


class ContinuousWalkingTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Continuous Walking Task')

        self.planner = ContinuousWalkingTaskPlanner(robotSystem)
        self.fitter = ImageFitter(self.planner)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def addButtons(self):

        self.addManualButton('plan one step', self.planner.planOneStep)
        self.addManualButton('plan two steps', self.planner.planTwoSteps)
        self.addManualSpacer()


    def addDefaultProperties(self):
        self.params.addProperty('Auto walk', False)

    def onPropertyChanged(self, propertySet, propertyName):
        pass

    def addTasks(self):
        pass


