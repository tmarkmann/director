from director import segmentation
from director.segmentation import *
from director.simpletimer import SimpleTimer

def fit2x4():
    dims = [0.0762, 0.0381]
    segmentation.startInteractiveLineDraw(dims)


def deepCopy(polyData):
    pd = vtk.vtkPolyData()
    pd.DeepCopy(polyData)
    return pd


from director.thirdparty import qhull_2d
from director.thirdparty import min_bounding_rect


def get2DAsPolyData(xy_points):
    '''
    Convert a 2D np array to a 3D polydata by appending z=0
    '''
    d = np.vstack((xy_points.T, np.zeros( xy_points.shape[0]) )).T
    d2=d.copy()
    return vtkNumpy.getVtkPolyDataFromNumpyPoints( d2 )


def computeHull(polyData):

    polyDataCentroid = segmentation.computeCentroid(polyData)
    pts = vnp.getNumpyFromVtk(polyData , 'Points')

    xy_points =  pts[:,[0,1]]

    om.getOrCreateContainer('hull points', parentObj=om.findObjectByName('debug'))
    vis.showPolyData(get2DAsPolyData(xy_points) , 'xy_points', parent='debug', visible=False)

    hull_points = qhull_2d.qhull2D(xy_points)
    vis.showPolyData( get2DAsPolyData(hull_points) , 'hull_points', parent='debug', visible=False).setProperty('Point Size', 5)
    # Reverse order of points, to match output from other qhull implementations
    hull_points = hull_points[::-1]

    # Find minimum area bounding rectangle
    (rot_angle, rectArea, rectDepth, rectWidth, center_point, corner_points_ground) = min_bounding_rect.minBoundingRect(hull_points)

    cornerPoints = np.vstack((corner_points_ground.T, polyDataCentroid[2]*np.ones( corner_points_ground.shape[0]) )).T
    cornerPolyData = vtkNumpy.getVtkPolyDataFromNumpyPoints(cornerPoints)

    vis.showPolyData( cornerPolyData, 'corner points', parent='debug', visible=False).setProperty('Point Size', 5)

    cornerTransform = transformUtils.frameFromPositionAndRPY(polyDataCentroid , [0,0, np.rad2deg(rot_angle) ] )

    return rot_angle, rectDepth, rectWidth, center_point


def sortClustersByDistanceToViewOrigin(clusters):

    viewFrame = SegmentationContext.getGlobalInstance().getViewFrame()
    viewOrigin = np.array(viewFrame.GetPosition())
    dists = [np.linalg.norm(viewOrigin - computeCentroid(cluster)) for cluster in clusters]
    clusters = [clusters[i] for i in np.argsort(dists)]
    return clusters


def processSurfaceClusters(clusters):

    viewFrame = SegmentationContext.getGlobalInstance().getViewFrame()
    viewOrigin = np.array(viewFrame.GetPosition())
    groundHeight = SegmentationContext.getGlobalInstance().getGroundHeight()

    for i, c in enumerate(clusters):
        vis.showPolyData(c, 'cluster %d' % i, color=getRandomColor())
        rot_angle, rectDepth, rectWidth, center_point = computeHull(c)

        zmax = np.median(vnp.getNumpyFromVtk(c, 'Points')[:,2])
        zmin = groundHeight

        origin = [center_point[0], center_point[1], (zmin+zmax)/2.0]

        dims = [rectDepth, rectWidth, zmax-zmin]
        dims = [float(x) for x in dims]

        frame = transformUtils.frameFromPositionAndRPY(origin , [0, 0, np.rad2deg(rot_angle)])

        pose = transformUtils.poseFromTransform(frame)
        desc = dict(classname='BoxAffordanceItem', Name='box', Dimensions=dims, pose=pose)
        affordanceManager.newAffordanceFromDescription(desc)


def fitBoxes(polyData):

    t = SimpleTimer()

    viewFrame = SegmentationContext.getGlobalInstance().getViewFrame()
    viewOrigin = np.array(viewFrame.GetPosition())
    groundHeight = SegmentationContext.getGlobalInstance().getGroundHeight()

    #removeGroundFunc = removeGroundSimple
    removeGroundFunc = removeGround
    t.reset()
    groundPoints, scenePoints =  removeGroundFunc(polyData, groundThickness=0.03, sceneHeightFromGround=0.05)
    print 'remove ground time:', t.elapsed()

    t.reset()
    vis.showPolyData(groundPoints, 'ground', color=[0,1,0], parent='debug')
    vis.showPolyData(scenePoints, 'scene', color=[1,0,1], parent='debug')
    print 'show poly data time', t.elapsed()

    t.reset()
    searchRegion = thresholdPoints(scenePoints, 'dist_to_plane', [0.05, 0.8])
    searchRegion = applyVoxelGrid(searchRegion, leafSize=0.02)
    print 'threshold search region time', t.elapsed()

    vis.showPolyData(searchRegion, 'search region voxel grid', color=[1,0,1], visible=False, parent='debug')

    t.reset()
    clusters = segmentation.findHorizontalSurfaces(searchRegion, removeGroundFirst=False, minClusterSize=20)
    print 'find surfaces time:', t.elapsed()

    t.reset()
    clusters = sortClustersByDistanceToViewOrigin(clusters)
    print 'sort clusters time:', t.elapsed()

    print '%d clusters' % len(clusters)

    t.reset()
    processSurfaceClusters(clusters)
    print 'fit rectangles time per cluster:', t.elapsed()/len(clusters)


def reorientBlocks(blocks, referenceAxis):

    forward = referenceAxis

    for block in blocks:

        blockFrame = block.getChildFrame().transform
        axes = transformUtils.getAxesFromTransform(blockFrame)
        origin = blockFrame.GetPosition()
        axes = [np.array(axis) for axis in axes]
        dims = block.getProperty('Dimensions')
        axisIndex, axis, sign = transformUtils.findTransformAxis(blockFrame, forward)
        if axisIndex == 2:
            continue

        if axisIndex == 0 and sign < 0:
            axes = [-axes[0], -axes[1], axes[2]]
        elif axisIndex == 1:
            dims = [dims[1], dims[0], dims[2]]
            if sign > 0:
                axes = [axes[1], -axes[0], axes[2]]
            else:
                axes = [-axes[1], axes[0], axes[2]]

        t = transformUtils.getTransformFromAxesAndOrigin(axes[0], axes[1], axes[2], origin)
        block.getChildFrame().copyFrame(t)
        block.setProperty('Dimensions', dims)


def getBoxObjects():
    return [obj for obj in om.getObjects() if obj.getProperty('Name') == 'box']

