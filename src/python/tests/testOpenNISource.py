from director.consoleapp import ConsoleApp


import director.visualization as vis
import director.objectmodel as om
import director.vtkAll as vtk
from director.shallowCopy import shallowCopy
from director.timercallback import TimerCallback

def startOpenNi():

    global source, obj, t

    source = vtk.vtkPCLOpenNISource()

    source.Update()
    p = source.GetOutput()
    obj = vis.showPolyData(shallowCopy(p), 'openni source')

    t = vtk.vtkTransform()
    t.PostMultiply()
    t.RotateX(-90)
    t.RotateZ(-90)
    obj.actor.SetUserTransform(t)

    obj.initialized = False

    def updateSource():
        source.Poll()
        source.Update()
        p = source.GetOutput()

        if not p.GetNumberOfPoints():
            return

        obj.setPolyData(shallowCopy(p))

        if not obj.initialized:
            obj.setProperty('Color By', 'rgb_colors')
            obj.initialized = True

        #print source.GetOutput().GetNumberOfPoints()


    global timerCallback
    timerCallback = TimerCallback(targetFps=30)
    timerCallback.callback = updateSource
    timerCallback.start()

    source.StartGrabber()


def main():

    global app, view

    app = ConsoleApp()

    app.setupGlobals(globals())
    app.showPythonConsole()

    view = app.createView()
    view.show()


    startOpenNi()

    app.start()



if __name__ == '__main__':
    main()
