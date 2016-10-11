import os
from director import ioUtils
from director import vtkNumpy as vnp
from director import vtkAll as vtk
from director import visualization as vis
from director import filterUtils
from director import transformUtils
from director import segmentation
from director import objectmodel as om
from director.timercallback import TimerCallback
from director.debugVis import DebugData
from director.consoleapp import ConsoleApp


import numpy as np


def loadPointCloud():
    #inputData = os.path.expanduser('~/Desktop/ground_non_convex_shape.vtp')
    inputData = os.path.expanduser('~/Desktop/wall_non_convex.vtp')
    pd = ioUtils.readPolyData(inputData)
    return pd


def pointsToOutlinePolyData(pts):

    d = DebugData()
    for i in xrange(len(pts)-1):
        d.addLine(pts[i], pts[i+1])
    d.addLine(pts[-1], pts[0])
    return d.getPolyData()


def writePlyForACD(polyData, filename):

    pts = vnp.getNumpyFromVtk(polyData, 'Points')
    numPoints = len(pts)

    f = open(filename, 'w')
    f.write('1\n')
    f.write('%d out\n' % numPoints)
    for p in pts[:numPoints]:
      f.write('%f %f\n' % (p[0], p[1]))
    f.write(' '.join([str(i+1) for i in xrange(numPoints)]))
    f.write('\n')
    f.close()

def readPlyFromACD(filename):

    f = open(filename)

    numShapes = int(f.readline().split()[0])
    shapes = []
    for shapeId in xrange(numShapes):
        pts = []
        numPoints = int(f.readline().split()[0])
        for pointId in xrange(numPoints):
            pt = [float(x) for x in f.readline().split()] + [0.0]
            assert len(pt) == 3
            pts.append(pt)

        ptIds = [int(x) for x in f.readline().split()]
        shapes.append(pts)

    return [pointsToOutlinePolyData(shape) for shape in shapes]


def showACDOutline(filename, transform):

    folder = om.getOrCreateContainer(os.path.basename(filename))

    concaveFolder = om.getOrCreateContainer('concave', parentObj=folder)
    convexFolder = om.getOrCreateContainer('convex', parentObj=folder)


    def computeConvexHull(pd):
        plane = vtk.vtkPlane()
        convexHull = vtk.vtkPolyData()
        vtk.vtkSurfaceFitter.ComputePlane(pd, plane)
        vtk.vtkSurfaceFitter.ComputeConvexHull(pd, plane, convexHull)
        return pointsToOutlinePolyData(vnp.getNumpyFromVtk(convexHull, 'Points'))
        #return convexHull

    for i, pd in enumerate(readPlyFromACD(filename)):
        color = vis.getRandomColor()
        pd = filterUtils.transformPolyData(pd, transform)
        vis.showPolyData(pd, 'shape %d' % i, parent=concaveFolder, color=color)
        co = vis.showPolyData(computeConvexHull(pd), 'shape %d' % i, parent=convexFolder, color=color)
        #co.setProperty('Surface Mode', 'Surface with edges')
        co.actor.GetProperty().SetLineWidth(5)

pd = loadPointCloud()


#pd = segmentation.applyVoxelGrid(pd, leafSize=0.02)
#pd = segmentation.labelOutliers(pd, searchRadius=0.1, neighborsInSearchRadius=4)
#pd = segmentation.thresholdPoints(pd, 'is_outlier', [0, 0])


plane = vtk.vtkPlane()

vtk.vtkSurfaceFitter.ComputePlane(pd, plane)

#normal = -np.array(plane.GetNormal())
#plane.SetNormal(normal)

convexHull = vtk.vtkPolyData()
vtk.vtkSurfaceFitter.ComputeConvexHull(pd, plane, convexHull)

alpha = float(sys.argv[1])

concaveHull = vtk.vtkPolyData()
vtk.vtkSurfaceFitter.ComputeConcaveHull(pd, plane, concaveHull, alpha)

centroid = filterUtils.computeCentroid(concaveHull)
t = transformUtils.getTransformFromOriginAndNormal(centroid, plane.GetNormal())

concaveHullTransformed = filterUtils.transformPolyData(concaveHull, t.GetLinearInverse())
writePlyForACD(concaveHullTransformed, 'shape.ply')

concaveHull = concaveHullTransformed
pd = filterUtils.transformPolyData(pd, t.GetLinearInverse())


pts = vnp.getNumpyFromVtk(concaveHull, 'Points')
pts[:,2] += 0.1
concaveHull = pointsToOutlinePolyData(pts)


app = ConsoleApp()
view = app.createView()

hullObj = vis.showPolyData(concaveHull, 'concave hull', color=[0,0,0])
obj = vis.showPolyData(pd, 'point cloud', color=[1,0,1])
obj.setProperty('Point Size', 3)
#obj.setProperty('Alpha', 0.5)
hullObj.actor.GetProperty().SetLineWidth(5)


#showACDOutline(os.path.expanduser('copy_shape.ply'), t)
showACDOutline(os.path.expanduser('copy_shape.ply'), vtk.vtkTransform())



view.camera().SetPosition(-0.1,0,1)
view.camera().SetFocalPoint(0,0,0)
view.camera().SetViewUp([1,-0.02,0])
view.resetCamera()

view.camera().SetPosition(-0.29973406436751693, 0.10462820529937744, 7.803231164709765)
view.camera().SetFocalPoint(0.4783785343170166, 0.10462820529937744, 0.022105177864432335)
view.forceRender()

def saveScreenshot():

    from director import screengrabberpanel
    screengrabberpanel.saveScreenshot(view, 'concave_hull_alpha_%.2f.png' % alpha)
    #app.quit()

app.viewOptions.setProperty('Gradient background', False)
app.viewOptions.setProperty('Background color', [1,1,1])
app.viewOptions.setProperty('Orientation widget', False)
app.gridObj.setProperty('Visible', False)

tc = TimerCallback()
tc.callback = saveScreenshot
tc.singleShot(1.0)

view.showMaximized()
app.start()
