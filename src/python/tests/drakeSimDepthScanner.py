from director import consoleapp
from director import mainwindowapp
from director import drakevisualizer
from director import depthscanner
from director import affordancemanager
from director import objectmodel
from director import transformUtils
from director import filterUtils
from director import ioUtils
from director import visualization as vis
from director import vtkAll as vtk
from director import applogic
from director.debugVis import DebugData

import numpy as np
import PythonQt
from PythonQt import QtGui, QtCore


def getCameraFrustumMesh(view, rayLength=1.0):

    origin = np.array(view.camera().GetPosition())

    def getCameraRay(displayPoint):
        _, ray = vis.getRayFromDisplayPoint(view, displayPoint)
        ray = ray-origin
        ray /= np.linalg.norm(ray)
        return ray

    viewWidth, viewHeight = view.renderWindow().GetSize()

    rays = [getCameraRay(x) for x in [[0.0, 0.0], [viewWidth, 0.0], [viewWidth, viewHeight], [0.0, viewHeight]]]

    rays = [rayLength*r for r in rays]
    camPos = origin
    lineRadius = 0.0
    color = [1.0, 1.0, 1.0]

    d = DebugData()
    d.addLine(camPos, camPos+rays[0], radius=lineRadius, color=color)
    d.addLine(camPos, camPos+rays[1], radius=lineRadius, color=color)
    d.addLine(camPos, camPos+rays[2], radius=lineRadius, color=color)
    d.addLine(camPos, camPos+rays[3], radius=lineRadius, color=color)
    d.addLine(camPos+rays[0], camPos+rays[1], radius=lineRadius, color=color)
    d.addLine(camPos+rays[1], camPos+rays[2], radius=lineRadius, color=color)
    d.addLine(camPos+rays[2], camPos+rays[3], radius=lineRadius, color=color)
    d.addLine(camPos+rays[3], camPos+rays[0], radius=lineRadius, color=color)
    pd = d.getPolyData()
    return pd


class CameraFrameSync(object):

    def __init__(self, camera, frameObj, renderFunction):
        self.frame = frameObj
        self.camera = camera
        self.renderFunction = renderFunction

        self._block = False

        def onCameraModified(obj, event):
            self.onCameraModified()

        self.observer = self.camera.AddObserver('ModifiedEvent', onCameraModified)
        self.frame.connectFrameModified(self.onFrameModified)

    def onCameraModified(self):
        if self._block:
            return
        self.setFrameFromCamera()

    def onFrameModified(self, frame):
        if self._block:
            return
        self.setCameraFromFrame()

    def getCameraTransform(self):
        return transformUtils.getLookAtTransform(
                  self.camera.GetFocalPoint(),
                  self.camera.GetPosition(),
                  self.camera.GetViewUp())

    def setCameraFromFrame(self):
        origin = np.array(self.frame.transform.GetPosition())
        axes = transformUtils.getAxesFromTransform(self.frame.transform)

        self._block = True
        self.camera.SetPosition(origin)
        self.camera.SetFocalPoint(origin+axes[0])
        self.camera.SetViewUp(axes[2])
        self.renderFunction()
        self._block = False

    def setFrameFromCamera(self):
        self._block = True
        self.frame.copyFrame(self.getCameraTransform())
        self._block = False


class CameraAffordance(object):

    def __init__(self, view, cameraName, affordanceManager):

        self.view = view

        d = DebugData()
        d.addCube(dimensions=[0.04, 0.08, 0.06], center=[-0.02, 0.0, 0.0], color=[1,0.5,0])
        d.addLine([0.0, 0.0, 0.0], [0.01, 0.0, 0.0], radius=0.023, color=[1,0.5,0])
        self.cameraModelMesh = d.getPolyData()

        desc = dict(classname='MeshAffordanceItem', Name=cameraName,
                      pose=transformUtils.poseFromTransform(vtk.vtkTransform()),
                      Filename='', ColorByName='RGB255', Visible=False)

        self.aff = affordanceManager.newAffordanceFromDescription(desc)

        self.cameraFrameSync = CameraFrameSync(self.view.camera(), self.aff.getChildFrame(), self.view.forceRender)

        self.eventFilter = PythonQt.dd.ddPythonEventFilter()
        self.view.installEventFilter(self.eventFilter)
        self.eventFilter.addFilteredEventType(QtCore.QEvent.Resize)
        self.eventFilter.connect('handleEvent(QObject*, QEvent*)', self.onResizeEvent)

        self.onResizeEvent()
        self.cameraFrameSync.setFrameFromCamera()

    def onResizeEvent(self):
        cameraMesh = getCameraFrustumMesh(self.view)
        cameraMesh = filterUtils.transformPolyData(cameraMesh, self.cameraFrameSync.getCameraTransform().GetLinearInverse())
        cameraMesh = filterUtils.appendPolyData([cameraMesh, self.cameraModelMesh])
        meshId = self.aff.getMeshManager().add(cameraMesh)
        self.aff.setProperty('Filename', meshId)


app = mainwindowapp.MainWindowAppFactory().construct()
app.gridObj.setProperty('Visible', True)
app.viewOptions.setProperty('Orientation widget', False)
app.viewOptions.setProperty('View angle', 45)
app.sceneBrowserDock.setVisible(False)
app.propertiesDock.setVisible(False)
app.mainWindow.setWindowTitle('Drake Visualizer')
app.mainWindow.show()
app.mainWindow.resize(920,600)
app.mainWindow.move(0,0)

view = app.view
view.setParent(None)
mdiArea = QtGui.QMdiArea()
app.mainWindow.setCentralWidget(mdiArea)
subWindow = mdiArea.addSubWindow(view)
subWindow.setMinimumSize(300,300)
subWindow.setWindowTitle('Camera image')
mdiArea.tileSubWindows()


drakeVis = drakevisualizer.DrakeVisualizer(view)
affordanceManager = affordancemanager.AffordanceObjectModelManager(view)
cameraAff = CameraAffordance(view, 'depth sensor', affordanceManager)

depthScanner = depthscanner.DepthScanner(view)
depthScanner.update()

dock = app.app.addWidgetToDock(depthScanner.imageView.view, QtCore.Qt.RightDockWidgetArea)
dock.setMinimumWidth(300)
dock.setMinimumHeight(300)

dock = app.app.addWidgetToDock(depthScanner.pointCloudView, QtCore.Qt.RightDockWidgetArea)
dock.setMinimumWidth(300)
dock.setMinimumHeight(300)

app.app.start(restoreWindow=False)
