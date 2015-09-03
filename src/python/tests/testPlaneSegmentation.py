import os
import math
from ddapp.consoleapp import ConsoleApp
from ddapp import ioUtils
from ddapp import segmentation
from ddapp import segmentationroutines
from ddapp import applogic
from ddapp import visualization as vis
from ddapp import vtkNumpy as vnp
from ddapp.shallowCopy import shallowCopy
from ddapp import vtkAll as vtk

import numpy as np


app = ConsoleApp()

# create a view
view = app.createView()
segmentation._defaultSegmentationView = view
#segmentation.initAffordanceManager(view)

#groundHeight = 0.0
#viewFrame = segmentation.transformUtils.frameFromPositionAndRPY([0, 0, groundHeight+2.0], [0, 0, 0])
#segmentationroutines.SegmentationContext.initWithUser(groundHeight, viewFrame)



# load poly data
dataDir = app.getTestingDataDirectory()
#polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/tilted_steps_lidar.pcd'))
polyData = ioUtils.readPolyData(os.path.join(dataDir, 'terrain/multisense_sweep.vtp'))
#vis.showPolyData(polyData, 'pointcloud snapshot')

groundPoints, scenePoints =  segmentation.removeGround(polyData)

vis.showPolyData(groundPoints, 'ground', visible=False)

polyData = segmentation.applyVoxelGrid(scenePoints, leafSize=0.02)


testNormals = False
if testNormals:

    print 'computing normals...'
    f = vtk.vtkRobustNormalEstimator()
    f.SetInput(polyData)
    f.SetMaxIterations(100)
    f.SetMaxEstimationError(0.01)
    f.SetMaxCenterError(0.02)
    f.SetRadius(0.1)



    f.Update()
    polyData = shallowCopy(f.GetOutput())
    polyData.GetPointData().SetNormals(polyData.GetPointData().GetArray('normals'))
    print 'done.'


    # filter points without normals
    normals = vnp.getNumpyFromVtk(polyData, 'normals')


    segmentation.flipNormalsWithViewDirection(polyData, [1, -1, -1])


    normalsValid = np.any(normals, axis=1)
    vnp.addNumpyToVtk(polyData, np.array(normalsValid, dtype=np.int32), 'normals_valid')

    vis.showPolyData(polyData, 'scene points', colorByName='normals_valid', visible=False)

    numPoints = polyData.GetNumberOfPoints()

    polyData = segmentation.thresholdPoints(polyData, 'normals_valid', [1, 1])
    vis.showPolyData(polyData, 'cloud normals', visible=True)

    print 'number of filtered points:', numPoints - polyData.GetNumberOfPoints()



    showGlyphs = False
    if showGlyphs:
        arrows = segmentation.applyArrowGlyphs(polyData, computeNormals=False)
        disks = segmentation.applyDiskGlyphs(polyData, computeNormals=False)

        #arrows2 = segmentation.applyArrowGlyphs(polyData)

        vis.showPolyData(arrows, 'arrows')
        vis.showPolyData(disks, 'disks')
        #vis.showPolyData(arrows2, 'arrows pcl normals')



f = vtk.vtkSurfaceFitter()
f.SetInput(polyData)

f.SetMaxError(0.03)
f.SetMaxAngle(np.radians(15))
f.SetSearchRadius(0.02)
f.SetMinimumNumberOfPoints(100)


f.Update()
polyData = shallowCopy(f.GetOutput())

labels = vnp.getNumpyFromVtk(polyData, 'plane_segmentation_labels')
maxLabel = labels.max()

polyData = segmentation.thresholdPoints(polyData, 'plane_segmentation_labels', [1, maxLabel])
vis.showPolyData(polyData, 'segmented cloud', colorByName='plane_segmentation_labels', visible=False)


for i in xrange(1, maxLabel+1):
    planePoints = segmentation.thresholdPoints(polyData, 'plane_segmentation_labels', [i, i])
    vis.showPolyData(planePoints, 'plane %d' % i, color=segmentation.getRandomColor())

    chull = vtk.vtkPolyData()
    plane = vtk.vtkPlane()

    f.ComputePlane(planePoints, plane)
    f.ComputeConvexHull(planePoints, plane, chull)
    chull = vis.showPolyData(chull, 'convex hull %d' % i)
    chull.setProperty('Surface Mode', 'Wireframe')

applogic.resetCamera([1,1,0])

if app.getTestingInteractiveEnabled():
    view.show()
    view.resize(1280, 1024)
    app.start()
