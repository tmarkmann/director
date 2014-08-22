from director.consoleapp import ConsoleApp


import director.visualization as vis
import director.objectmodel as om
import director.vtkAll as vtk
from director.shallowCopy import shallowCopy
from director.timercallback import TimerCallback
import director.vtkVelodyneHDLPython as vv


def startVelodyne():

    global source, obj, t

    source = vv.vtkVelodyneHDLSource()

    #source.SetCorrectionsFile('/Users/pat/Desktop/robotx/cal.xml')
    source.Update()

    p = source.GetOutput()
    obj = vis.showPolyData(shallowCopy(p), 'velodyne source')

    #t = vtk.vtkTransform()
    #t.PostMultiply()
    #t.RotateX(-90)
    #t.RotateZ(-90)
    #obj.actor.SetUserTransform(t)

    obj.initialized = False

    def updateSource():
        source.Poll()
        source.UpdateInformation()

        e = source.GetExecutive()
        inf = e.GetOutputInformation().GetInformationObject(0)
        numTimeSteps = e.TIME_STEPS().Length(inf)
        if not numTimeSteps:
            return

        lastTimeStep = e.TIME_STEPS().Get(inf, numTimeSteps-1)
        e.SetUpdateTimeStep(0, lastTimeStep)

        source.Update()
        p = source.GetOutput()

        if not p.GetNumberOfPoints():
            return

        obj.setPolyData(shallowCopy(p))

        if not obj.initialized:
            del obj.rangeMap['intensity']
            obj.setProperty('Color By', 'intensity')
            obj.mapper.SetColorModeToMapScalars()
            obj.initialized = True

        #print source.GetOutput().GetNumberOfPoints()


    global timerCallback
    timerCallback = TimerCallback(targetFps=30)
    timerCallback.callback = updateSource
    timerCallback.start()
    source.Start()


def main():

    global app, view

    app = ConsoleApp()

    app.setupGlobals(globals())
    app.showPythonConsole()

    view = app.createView()
    view.show()


    startVelodyne()

    app.start()



if __name__ == '__main__':
    main()
