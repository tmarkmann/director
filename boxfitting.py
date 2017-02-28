from director import segmentation
from director import segmentationroutines
from director.simpletimer import SimpleTimer
from director.shallowCopy import shallowCopy
from director import affordancemanager
from director import visualization as vis
from director import filterUtils
from director import transformUtils
from director import vtkNumpy as vnp
from director import vtkAll as vtk
from director import cameraview
from director.fieldcontainer import FieldContainer
from director.debugVis import DebugData
from director import filterUtils
from director.thirdparty import qhull_2d
from director.thirdparty import min_bounding_rect

import numpy as np
import os
import itertools
from scipy.sparse.csgraph import connected_components


def computePlanarConvexHull(polyData, expectedNormal=None):

    plane = vtk.vtkPlane()
    vtk.vtkSurfaceFitter.ComputePlane(polyData, plane)

    if expectedNormal is not None:
        planeNormal = plane.GetNormal()
        if np.dot(planeNormal, expectedNormal) < 0:
            plane.SetNormal(-1*np.array(planeNormal))

    chull = vtk.vtkPolyData()
    vtk.vtkSurfaceFitter.ComputeConvexHull(polyData, plane, chull)
    return FieldContainer(points=polyData, convexHull=chull, plane=plane, origin=np.array(plane.GetOrigin()), normal=np.array(plane.GetNormal()))


def regionGrowingPlanarSegmentation(polyData):

    f = vtk.vtkSurfaceFitter()
    f.SetInput(polyData)

    f.SetMaxError(0.005)
    f.SetMaxAngle(np.radians(3))
    f.SetSearchRadius(0.01)
    f.SetMinimumNumberOfPoints(10)

    ne = f.GetRobustNormalEstimator()
    ne.SetRadius(0.01)
    ne.SetMaxEstimationError(0.01)
    ne.SetMaxCenterError(0.01)

    f.Update()
    polyData = shallowCopy(f.GetOutput())

    labels = vnp.getNumpyFromVtk(polyData, 'plane_segmentation_labels')
    maxLabel = labels.max()

    planes = []
    for i in xrange(1, maxLabel+1):
        planes.append(segmentation.thresholdPoints(polyData, 'plane_segmentation_labels', [i, i]))

    return planes


def get2DAsPolyData(pts):
    '''
    Convert a 2D np array to a 3D polydata by appending z=0
    '''
    d = np.vstack((pts.T, np.zeros(pts.shape[0]))).T
    return vnp.getVtkPolyDataFromNumpyPoints(d.copy())


def computeMinimumBoundingRectangle(polyData, origin, normal):

    polyDataCentroid = origin

    t = transformUtils.getTransformFromOriginAndNormal(origin, normal)
    polyData = filterUtils.transformPolyData(polyData, t.GetLinearInverse())

    pts = vnp.getNumpyFromVtk(polyData , 'Points')

    xyPoints =  pts[:,[0,1]]

    vis.showPolyData(get2DAsPolyData(xyPoints), 'xy points', parent='debug', visible=False)

    hullPoints = qhull_2d.qhull2D(xyPoints)

    vis.showPolyData(get2DAsPolyData(hullPoints), 'hull points', parent='debug', visible=False).setProperty('Point Size', 5)

    # Reverse order of points, to match output from other qhull implementations
    hullPoints = hullPoints[::-1]

    # Find minimum area bounding rectangle
    rotAngle, rectArea, rectWidth, rectHeight, centerPoint, cornerPoints = min_bounding_rect.minBoundingRect(hullPoints)


    cornerPolyData = get2DAsPolyData(cornerPoints)
    vis.showPolyData(cornerPolyData, 'corner points', parent='debug', visible=False).setProperty('Point Size', 5)

    originToPts = transformUtils.frameFromPositionAndRPY([centerPoint[0], centerPoint[1], 0.0], [0,0, np.rad2deg(rotAngle)])

    points = filterUtils.transformPolyData(get2DAsPolyData(xyPoints), originToPts.GetLinearInverse())
    pointsToWorld = transformUtils.concatenateTransforms([originToPts, t])

    obj = vis.showPolyData(points, 'face points', parent='debug', visible=False)
    vis.addChildFrame(obj)
    obj.getChildFrame().copyFrame(pointsToWorld)
    #obj.getChildFrame().setProperty('Visible', True)

    d = DebugData()
    origin = np.zeros(3)
    d.addSphere(origin, radius=0.001)
    d.addLine(origin, [rectWidth/2.0, 0.0, 0.0], radius=0.001)
    d.addLine(origin, [0.0, rectHeight/2.0, 0.0], radius=0.001)
    vis.showPolyData(d.getPolyData(), 'lines debug', visible=False, parent=obj).actor.SetUserTransform(obj.getChildFrame().transform)

    return FieldContainer(
        points=points,
        frame=pointsToWorld,
        width=rectWidth,
        height=rectHeight,
        origin=np.zeros(3),
        xaxis=np.array([1.0, 0.0, 0.0]),
        yaxis=np.array([0.0, 1.0, 0.0]),
        zaxis=np.array([0.0, 0.0, 1.0]),
        )


def getCameraToWorld():
    cameraName = 'OPENNI_FRAME_LEFT'
    q = cameraview.imageManager.queue
    utime = q.getCurrentImageTime(cameraName)
    t = vtk.vtkTransform()
    q.getTransform(cameraName, 'local', utime, t)
    return t


def setSegmentationContextFromCameraFrame():
    t = getCameraToWorld()
    vis.updateFrame(t, 'view frame', visible=False)
    segmentationroutines.SegmentationContext._globalSegmentationContext = None
    segmentationroutines.SegmentationContext.initWithUser(0.0, t, viewAxis=2)


def checkFaceCompatibility(face1, face2, name, threshold=0.02):


    normalDot = np.dot(face1.normal, face2.normal)

    t1 = face1.rectData.frame
    t2 = face2.rectData.frame

    origin2In1 = np.array(t1.GetLinearInverse().TransformPoint(t2.GetPosition()))


    x, y, _ = origin2In1
    halfWidth = face1.rectData.width*0.5
    halfHeight = face1.rectData.height*0.5

    alignedX = -(halfWidth+threshold) <= x <= (halfWidth+threshold)
    alignedY = -(halfHeight+threshold) <= y <= (halfHeight+threshold)

    isMatch = alignedX and alignedY and np.abs(normalDot) < 0.01

    d = DebugData()
    origin = np.zeros(3)
    d.addSphere(np.zeros(3), radius=0.002)
    d.addLine(origin,[halfWidth, 0.0, 0.0])
    d.addLine(origin,[0.0, halfHeight, 0.0])
    d.addLine(origin,origin2In1)

    d.addSphere([halfWidth+threshold, 0.0, 0.0], radius=0.002, color=[1,0,0])
    d.addSphere([0.0, halfHeight+threshold, 0.0], radius=0.002, color=[0,1,0])
    d.addSphere(origin2In1, radius=0.002, color=[0,0,1])


    name += ' %0.2f' % normalDot

    if isMatch:
        name += '  (match)'
    vis.showPolyData(filterUtils.transformPolyData(d.getPolyData(), t1), name, colorByName='RGB255', parent='face debug')

    return isMatch



def makeBoxFromFaces(faces, affordanceManager):

    points = []
    t = faces[0].rectData.frame

    for face in faces:
        points.append(filterUtils.transformPolyData(face.points, t.GetLinearInverse()))

    boxPoints = filterUtils.appendPolyData(points)
    vis.showPolyData(boxPoints, 'appended box points', visible=False)

    bounds = boxPoints.GetBounds()
    corner1 = np.array([bounds[0], bounds[2], bounds[4]])
    corner2 = np.array([bounds[1], bounds[3], bounds[5]])
    center = (corner1+corner2)/2.0
    dims = list(corner2 - corner1)

    boxFrame = transformUtils.frameFromPositionAndRPY(center, [0,0,0])
    boxFrame = transformUtils.concatenateTransforms([boxFrame, t])

    print center
    print dims

    desc = dict(classname='BoxAffordanceItem',
                Name='box',
                Color=[0.33,0.66,1.0],
                pose=transformUtils.poseFromTransform(boxFrame),
                Dimensions=dims)
    box = affordanceManager.newAffordanceFromDescription(desc)
    box.getChildFrame().setProperty('Scale', 0.1)


def runBoxFitting(polyData, affordanceManager):


    viewDirection = segmentationroutines.SegmentationContext.getGlobalInstance().getViewDirection()

    vis.showPolyData(polyData, 'pointcloud', parent='debug', visible=False)


    polyData = segmentation.applyVoxelGrid(polyData, 0.005)
    vis.showPolyData(polyData, 'voxel pointcloud', parent='debug', visible=False)

    '''
    polyData, planeFilter = segmentation.removeMajorPlane(polyData, distanceThreshold=0.005)

    planePoints = segmentation.thresholdPoints(planeFilter.GetOutput(), 'ransac_labels', [1.0, 1.0])

    fields = computePlanarConvexHull(planePoints, expectedNormal=-viewDirection)
    vis.showPolyData(fields.convexHull, 'table convex hull', visible=False)

    d = DebugData()
    d.addSphere(fields.plane.GetOrigin(), radius=0.02)
    d.addArrow(fields.plane.GetOrigin(), np.array(fields.plane.GetOrigin()) + 0.2*np.array(fields.plane.GetNormal()))
    vis.showPolyData(d.getPolyData(), 'table debug', visible=False)

    vis.showPolyData(polyData, 'plane removed', visible=False)
    '''


    planes = regionGrowingPlanarSegmentation(polyData)
    planes = [computePlanarConvexHull(plane, expectedNormal=-viewDirection) for plane in planes]

    for i, obj in enumerate(planes):
        pobj = vis.showPolyData(obj.points, 'plane %d' % i, parent='plane segmentation', color=vis.getRandomColor())
        pobj.setProperty('Point Size', 5)
        rectData = computeMinimumBoundingRectangle(obj.points, obj.origin, obj.normal)
        obj._add_fields(rectData=rectData)

    numPlanes = len(planes)
    connections = np.zeros((numPlanes, numPlanes))

    for i, j in itertools.permutations(xrange(len(planes)), r=2):
        print i, j

        print 'checking plane %d and %d' % (i, j)
        if checkFaceCompatibility(planes[i], planes[j], name='check %d, %d' % (i, j)) and checkFaceCompatibility(planes[j], planes[i], name='check %d, %d' % (j, i)):
            print '  found match'
            connections[i][j] = 1.0
            connections[j][i] = 1.0


    nComponents, labels = connected_components(connections, connection='strong')

    boxes = []
    for i in xrange(nComponents):
        inds = np.where(labels==i)[0]
        print i, inds, len(inds)
        if len(inds) >= 2:
            boxes.append([planes[k] for k in inds])


    for box in boxes:
        makeBoxFromFaces(box, affordanceManager)
