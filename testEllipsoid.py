from director.debugVis import DebugData
from director import transformUtils
from director import filterUtils
from director import vtkAll as vtk
from director import visualization as vis
from director.timercallback import TimerCallback
import time
import math


def addEllipsoid(radii, name='ellipsoid'):

    # create unit sphere
    d = DebugData()
    d.addSphere(center=[0,0,0], radius=1.0)

    # scale sphere along x,y,z axes
    pd = d.getPolyData()
    t = vtk.vtkTransform()
    t.Scale(radii)
    pd = filterUtils.transformPolyData(pd, t)
    obj = vis.showPolyData(pd, name)
    vis.addChildFrame(obj)
    return obj



# demo animation code
def tick():

    # args are position, quaternion
    # there are other functions that take roll pitch yaw or coordinate axes
    startFrame = transformUtils.transformFromPose([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
    endFrame = transformUtils.transformFromPose([0.5, 0.5, 0.5], [1.0, 0.5, 0.5, 0.5])

    # compute a new transform as a function of time
    p = abs(math.sin(time.time()))
    newFrame = transformUtils.frameInterpolate(startFrame, endFrame, p)

    # reposition object (accesses global variable)
    global obj
    obj.getChildFrame().copyFrame(newFrame)


# draw ellipse and set color / transparency
obj = addEllipsoid(radii=[0.1, 0.2, 0.3])
obj.setProperty('Color', [0.0, 1.0, 0.0])
obj.setProperty('Alpha', 0.3)


# make an animation loop
tc = TimerCallback()
tc.callback = tick
tc.start()

# execute tc.stop() to stop animation

