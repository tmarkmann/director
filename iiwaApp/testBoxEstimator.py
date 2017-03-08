import os
import sys
import time
import copy
import imp

from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import robotsystem
from director import segmentation
from director import cameraview
from director import pydrakeik
from director import packagepath
from director import roboturdf
from director import fieldcontainer
from director import framevisualization
from director import drcargs

import PythonQt
from PythonQt import QtCore, QtGui


import drake as lcmdrake
import bot_core as lcmbotcore



def makeRobotSystem(view):

    factory = robotsystem.ComponentFactory()
    factory.register(robotsystem.RobotSystemFactory)
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, useSegmentationAffordances=True)
    robotSystem = factory.construct(view=view, options=options)

    return robotSystem


def initImageManager():
    imageManager = cameraview.ImageManager()
    cameraview.imageManager = imageManager
    return imageManager


def initDepthPointCloud(imageManager, view):

    openniDepthPointCloud = segmentation.DisparityPointCloudItem('openni point cloud', 'OPENNI_FRAME', 'OPENNI_FRAME_LEFT', imageManager)
    openniDepthPointCloud.addToView(view)
    om.addToObjectModel(openniDepthPointCloud, parentObj=om.findObjectByName('sensors'))
    openniDepthPointCloud.setProperty('Visible', True)
    openniDepthPointCloud.setProperty('Target FPS', 1e6)
    return openniDepthPointCloud


def newCameraView(imageManager, channelName='OPENNI_FRAME', cameraName='OPENNI_FRAME_LEFT', viewName='OpenNI Frame'):

    view = PythonQt.dd.ddQVTKWidgetView()
    view.orientationMarkerWidget().Off()
    view.backgroundRenderer().SetBackground([0,0,0])
    view.backgroundRenderer().SetBackground2([0,0,0])

    imageManager.queue.addCameraStream(channelName, cameraName, lcmbotcore.images_t.LEFT)
    imageManager.addImage(cameraName)

    cameraView = cameraview.CameraImageView(imageManager, cameraName, viewName=viewName, view=view)
    cameraView.eventFilterEnabled = False
    view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())

    return cameraView


def addToolBarAction(name, callback):
    toolBar = applogic.findToolBar('Main Toolbar')
    app.addToolBarAction(toolBar, name, icon='', callback=callback)


def getDrakeSimTimeForEvent(event):
    if event.channel == 'DRAKE_VIEWER_DRAW':
        msg = lcmbot.viewer_draw_t.decode(event.data)
        return msg.timestamp*1000


def initDrakeTimeDisplay():

    def onViewerDraw(msg):
        t = msg.timestamp*1e-3
        vis.updateText('sim time: %.3f' % t, 'sim time')

    lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmdrake.lcmt_viewer_draw, onViewerDraw)


#####################################################

import boxfitting
from director.lcmlogplayer import LcmLogPlayer
import bot_core as lcmbot
from director.asynctaskqueue import AsyncTaskQueue
from director.tasks import robottasks as rt


viewBackgroundLightHandler.setEnabled(True)

robotSystem = makeRobotSystem(view)

imageManager = initImageManager()
openniDepthPointCloud = initDepthPointCloud(imageManager, view)

openniDepthPointCloud.setProperty('Max Range', 30)
cameraView = newCameraView(imageManager)
#cameraView.view.show()

logFileName = os.path.expanduser('~/Desktop/logs/sim-box-drop-rotated-stack.lcmlog')
logPlayer = LcmLogPlayer()
logPlayer.readLog(logFileName, eventTimeFunction=getDrakeSimTimeForEvent)
logPlayer.skipToTime(0.0)
initDrakeTimeDisplay()


advanceTime = 1/100.0
def advanceLog():
    logPlayer.advanceTime(advanceTime)

def rewindLog():
    logPlayer.skipToTime(0.01)

def setupRecording():
    screenGrabberPanel.startRecording()
    screenGrabberPanel.recordTimer.stop()


frameCount = 0
maxFrames = 300
taskQueue = AsyncTaskQueue()

def recordNextFrame():
    global frameCount
    frameCount += 1
    if frameCount > maxFrames:
        return

    taskQueue.addTask(clear)
    taskQueue.addTask(test)
    taskQueue.addTask(screenGrabberPanel.onRecordTimer)
    taskQueue.addTask(advanceLog)
    taskQueue.addTask(rt.DelayTask(delayTime=0.01))
    taskQueue.addTask(recordNextFrame)
    taskQueue.start()


def stopRecording():
    global frameCount
    frameCount = 0
    taskQueue.stop()


def test():

    openniDepthPointCloud.update()

    boxfitting.setSegmentationContextFromCameraFrame()
    polyData = openniDepthPointCloud.polyData
    boxfitting.runBoxFitting(polyData, robotSystem.affordanceManager)
    om.findObjectByName('face debug').setProperty('Visible', False)
    om.findObjectByName('plane segmentation').setProperty('Visible', False)
    view.forceRender()


def clear():
    names = ['segmentation', 'debug', 'plane segmentation', 'affordances', 'face debug']
    for name in names:
        om.removeFromObjectModel(om.findObjectByName(name))


addToolBarAction('test', test)
addToolBarAction('clear', clear)
addToolBarAction('stop', stopRecording)

