import os
import sys

sys.path.append(os.path.join(os.environ['DRAKE_BASE'], 'build/install/lib/python2.7/dist-packages'))
sys.path.append(os.path.join(os.environ['DRAKE_BASE'], 'build/install/lib/python2.7/site-packages'))

from director import lcmUtils
from director import lcmframe
from director import transformUtils
from director import robotsystem
from director import segmentation
from director import cameraview

import mytaskpanel

import PythonQt
from PythonQt import QtCore, QtGui


import drake as lcmdrake
import bot_core as lcmbotcore


def setTagToWorld(pos, rpy):
    global tagToWorld
    tagToWorld = transformUtils.frameFromPositionAndRPY(pos, rpy)


def getTagToWorld():
    return tagToWorld


setTagToWorld([0.29,-0.38,-0.10], [-93,1,1.5])


def onAprilTagMessage(msg, channel):
    tagToCamera = lcmframe.frameFromRigidTransformMessage(msg)
    vis.updateFrame(tagToCamera, channel)

    cameraToTag = tagToCamera.GetLinearInverse()
    tagToWorld = getTagToWorld()
    cameraToWorld = transformUtils.concatenateTransforms([cameraToTag, tagToWorld])

    cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
    lcmUtils.publish('OPENNI_FRAME_LEFT_TO_LOCAL', cameraToWorldMsg)

    vis.updateFrame(vtk.vtkTransform(), 'world')
    vis.updateFrame(cameraToWorld, 'camera to world')
    vis.updateFrame(tagToWorld, 'tag to world')


lcmUtils.addSubscriber('APRIL_TAG_0218_TO_CAMERA_LEFT', lcmbotcore.rigid_transform_t, onAprilTagMessage, callbackNeedsChannel=True)


def setupKinect():
    from director import kinectlcm
    kinectlcm.init(view)

#setupKinect()



def makeRobotSystem(view):
    factory = robotsystem.RobotSystemFactory()
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True, useSegmentation=True, useSegmentationAffordances=True)
    robotSystem = factory.construct(view=view, options=options)


    # use pydrake ik backend
    ikPlanner = robotSystem.ikPlanner
    ikPlanner.planningMode = 'pydrake'
    ikPlanner.plannerPub._setupLocalServer()

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
    openniDepthPointCloud.setProperty('Target FPS', 30)
    return openniDepthPointCloud


def initCameraView(imageManager):

    view = PythonQt.dd.ddQVTKWidgetView()
    view.orientationMarkerWidget().Off()
    view.backgroundRenderer().SetBackground([0,0,0])
    view.backgroundRenderer().SetBackground2([0,0,0])

    imageManager.queue.addCameraStream('OPENNI_FRAME', 'OPENNI_FRAME_LEFT', lcmbotcore.images_t.LEFT)
    imageManager.addImage('OPENNI_FRAME_LEFT')

    cameraView = cameraview.CameraImageView(imageManager, 'OPENNI_FRAME_LEFT', viewName='OpenNI Frame', view=view)
    cameraView.eventFilterEnabled = False
    view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleImage())

    #view.show()
    return cameraView


def sendGripperCommand(targetPositionMM, force):
    msg = lcmdrake.lcmt_schunk_wsg_command()
    msg.utime = int(time.time()*1e6)
    msg.force = force
    msg.target_position_mm = targetPositionMM
    lcmUtils.publish('SCHUNK_WSG_COMMAND', msg)


def gripperOpen():
    sendGripperCommand(105, 40)


def gripperClose():
    sendGripperCommand(7, 40)


def setupToolbar():
    toolBar = applogic.findToolBar('Main Toolbar')
    app.app.addToolBarAction(toolBar, 'Gripper Open', icon='', callback=gripperOpen)
    app.app.addToolBarAction(toolBar, 'Gripper Close', icon='', callback=gripperClose)


##############################################

robotSystem = makeRobotSystem(view)

app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea)
app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea)

applogic.resetCamera(viewDirection=[-1,0,0], view=view)



imageManager = initImageManager()
openniDepthPointCloud = initDepthPointCloud(imageManager, view)
cameraView = initCameraView(imageManager)

taskPanel = mytaskpanel.MyTaskPanel(robotSystem, cameraView)
taskPanel.widget.show()
