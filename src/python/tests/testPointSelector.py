from director import consoleapp
from director import pointselector
from director import visualization as vis
from director import vtkNumpy as vnp
from director import vieweventfilter
from director import vtkAll as vtk
from director import objectmodel as om
from director.debugVis import DebugData

from PythonQt import QtCore

import numpy as np

app = consoleapp.ConsoleApp()
view = app.createView()

randomPoints = np.random.random((1000,3))*3.0
polyData = vnp.numpyToPolyData(randomPoints, createVertexCells=True)
vis.showPolyData(polyData, 'pointcloud')

view.show()
view.forceRender()

#selector = pointselector.PointSelector(view, polyData)
#selector.pickArea((200,200), (300,300))
#assert selector.getSelectedPoints().GetNumberOfPoints()

#view.renderWindow().GetInteractor().SetInteractorStyle(vtk.vtkInteractorStyleUnicam())

app.gridObj.actor.SetPickable(True)


class MyEventFilter(vieweventfilter.ViewEventFilter):

    def filterEvent(self, obj, event):
        if event.type() == QtCore.QEvent.MouseButtonRelease:
            om.removeFromObjectModel(om.findObjectByName('cor'))

        elif event.type() == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.RightButton:
          self.onRightMousePress(event)

        vieweventfilter.ViewEventFilter.filterEvent(self, obj, event)

    def onRightMousePress(self, event):
        self.updateMouseHitPoint(event)

    def onLeftMousePress(self, event):
        self.updateMouseHitPoint(event)


    def updateMouseHitPoint(self, event):

        cor = np.array(view.camera().GetFocalPoint())

        displayPoint = self.getMousePositionInView(event)
        worldPoint = [0,0,0,0]
        vtk.vtkInteractorObserver.ComputeDisplayToWorld(view.renderer(), displayPoint[0], displayPoint[1], 0.5, worldPoint)

        res = vis.pickPoint(displayPoint, self.view, pickType='cells')
        if res.pickedPoint is not None:
            print 'using pick point'
            worldPoint = res.pickedPoint

        #vec = np.array(worldPoint) - cor
        #view.camera().SetFocalPoint(cor + vec)
        #view.camera().SetPosition(np.array(view.camera().GetPosition()) + vec)

        d = DebugData()
        d.addSphere((0,0,0), radius=1.0)
        corObj = vis.showPolyData(d.getPolyData(), 'cor', color=[1,1,0])
        t = vtk.vtkTransform()
        t.Translate(worldPoint[:3])
        corObj.actor.SetUserTransform(t)

        print 'world point:', worldPoint

        iren = self.view.renderWindow().GetInteractor()
        style = iren.GetInteractorStyle()
        style.SetCustomCenterOfRotation(worldPoint[:3])


def onStartRender(renderWindow, event):

    obj = om.findObjectByName('cor')
    if not obj:
      return

    iren = view.renderWindow().GetInteractor()
    style = iren.GetInteractorStyle()

    t = obj.actor.GetUserTransform()
    scale = style.ComputeScale(t.GetPosition(), view.renderer())


    scale = scale*10

    tt = vtk.vtkTransform()
    tt.Translate(t.GetPosition())
    tt.Scale(scale, scale, scale)
    obj.actor.SetUserTransform(tt)


eventFilter = MyEventFilter(view)

observer = view.renderWindow().AddObserver('StartEvent', onStartRender)
app.start()
