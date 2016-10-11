pd = segmentation.getCurrentRevolutionData()

def getMultisenseLidarData():
    return segmentation.getCurrentRevolutionData()


def segmentViconMarkersFromLidar(polyData, minIntensity=3800):
    intensityPts = segmentation.thresholdPoints(polyData, 'intensity', [minIntensity, np.inf])
    clusters = segmentation.extractClusters(intensityPts, minClusterSize=3)

    for i, cluster in enumerate(clusters):
        xyz = segmentation.computeCentroid(cluster)
        t = transformUtils.frameFromPositionAndRPY(xyz, [0.0, 0.0, 0.0])
        vis.showFrame(t, 'lidar marker frame %d' % i, scale=0.2)


def getMarkerPoints(frameIds, nameTemplate='frame %d'):

    pts = np.zeros((len(frameIds), 3))

    for i, frameId in enumerate(frameIds):
        f = om.findObjectByName(nameTemplate % frameId)
        pts[i,:] = f.transform.GetPosition()

    return pts


def applyLandmarkTransform(ptsA, ptsB):

    sourcePoints = vnp.getVtkPointsFromNumpy(ptsA)
    targetPoints = vnp.getVtkPointsFromNumpy(ptsB)

    f = vtk.vtkLandmarkTransform()
    f.SetSourceLandmarks(sourcePoints)
    f.SetTargetLandmarks(targetPoints)
    f.SetModeToRigidBody()
    f.Update()

    mat = f.GetMatrix()
    t = vtk.vtkTransform()
    t.PostMultiply()
    t.SetMatrix(mat)
    return t


def transformLidar(polyData, lidarFrameIds=[5,4,2,6,3], viconFrameIds=[1,2,0,4,3]):

    lidarMarkerPoints = getMarkerPoints(lidarFrameIds, nameTemplate='lidar marker frame %d')
    viconMarkerPoints = getMarkerPoints(viconFrameIds, nameTemplate='OPT_j_VisCal marker frame %d')
    t = applyLandmarkTransform(lidarMarkerPoints, viconMarkerPoints)
    polyData = filterUtils.transformPolyData(polyData, t)
    vis.showPolyData(polyData, 'transformed lidar')

    return t


def getMarkerObjects():
    return [obj for obj in om.getObjects() if obj.getProperty('Name').startswith('manual marker ') and isinstance(obj, vis.FrameItem)]


def addMarker(xyz):

    numberOfMarkers = len(getMarkerObjects())
    d = DebugData()
    d.addSphere(np.zeros(3), radius=0.007)
    obj = vis.showPolyData(d.getPolyData(), 'manual marker %d' % (numberOfMarkers), color=segmentation.getRandomColor(), parent='manual markers')
    vis.addChildFrame(obj)
    t = transformUtils.frameFromPositionAndRPY(xyz, np.zeros(3))
    obj.getChildFrame().copyFrame(t)


def computeHeadToNeck():

    headToVicon = applyLandmarkTransform(lidarMarkerPoints, viconMarkerPoints)
    neckToVicon = applyLandmarkTransform(manualMarkerPoints, optMainMarkerPoints)


    vis.updateFrame(headToVicon, 'head to vicon')
    vis.updateFrame(neckToVicon, 'neck to vicon')

    headToNeck = transformUtils.concatenateTransforms([headToVicon, neckToVicon.GetLinearInverse()])



def pickerCallback(*pts):
    for p in pts:
        addMarker(p)


def makePointPicker():

    picker = PointPicker(view)
    picker.annotationFunc = pickerCallback
    picker.numberOfPoints = 1
    picker.pickType = 'cells'
    print 'remember to call picker.start()'
    return picker


def getRobotModelMesh():
    polyData = vtk.vtkPolyData()
    

def alignLinkWithWorldFrame(linkName='neck_link'):

    # move base to world origin
    robotStateJointController.q[:6] = np.zeros(6)
    robotStateJointController.push()

    # compute link frame
    linkFrame = robotStateModel.getLinkFrame(linkName)

    # move base by inverse of link frame
    rpy = transformUtils.rollPitchYawFromTransform(linkFrame.GetLinearInverse())
    xyz = linkFrame.GetLinearInverse().GetPosition()
    pose = np.hstack([xyz, rpy])
    robotStateJointController.q[:6] = pose
    robotStateJointController.push()

    return linkFrame


'''
Steps:

  # position robot at world origin, zero translation and rotation
  robotStateJointController.setZeroPose()

  # get head link frame in world
  headToWorld = robotStateModel.getLinkFrame('head')

  # get lidar data
  lidarInWorldFrame = segmentation.getCurrentRevolutionData()

  # get and show lidar in head frame
  lidarInHeadFrame = filterUtils.transformPolyData(lidarInWorldFrame, headToWorld.GetLinearInverse())
  vis.updatePolyData('lidar in head frame', lidarInHeadFrame)

  # segment markers using lidar in head frame)
  segmentViconMarkersFromLidar(lidarInHeadFrame)

  # compute headToVicon
  headToVicon = transformLidar(lidarInHeadFrame, ...)

  # show lidar in vicon frame
  lidarInViconFrame = filterUtils.transformPolyData(lidarInHeadFrame, headToVicon)
  vis.updatePolyData('lidar in vicon frame', lidarInViconFrame)


  # now repeat process for robot

  # set robot to zero pose
  robotStateJointController.setZeroPose()

  # compute neck link frame in world
  neckToWorld = robotStateModel.getLinkFrame('neck_link')

  # move robot so that neck as aligned to world origin
  alignLinkWithWorldFrame('neck_link')


  #apply manual markers
  picker = makePointPicker()
  picker.start()
  # ...

  # compute transform neckToVicon using correspondance between markers on robot and markers in vicon
  # compute headToNeck using headToVicon and neckToVicon

'''

