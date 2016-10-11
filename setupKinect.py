import bot_core as lcmbot

from director import kinectlcm
from director import segmentationroutines

kinectlcm.init(view)

kinectSource = om.findObjectByName('kinect source')
kinectSource.setProperty('Visible', False)

kinectFrame = vis.updateFrame(vtk.vtkTransform(), 'kinect camera frame', visible=False)
segmentationroutines.SegmentationContext._globalSegmentationContext = None
segmentationroutines.SegmentationContext.initWithUser(0.0, kinectFrame.transform, viewAxis=2)



def printViewDirection():

    print segmentationroutines.SegmentationContext.getGlobalInstance().getViewDirection()


tagToWorld = vtk.vtkTransform()
tagToWorld.PostMultiply()
tagToWorld.RotateX(-90)
tagToWorld.RotateZ(90)
tagToWorld.Translate([0.21, 0.33, -0.12])


def getKinectTransformed():
    return om.findObjectByName('kinect transformed')


def getPointcloudSnapshot():
    return filterUtils.shallowCopy(getKinectTransformed().polyData)


def onFrame(msg):
    pos = msg.trans
    quat = msg.quat
    tagToCamera = transformUtils.transformFromPose(pos, quat)


    pd = kinectSource.polyData
    if not pd.GetNumberOfPoints():
        return

    cameraToWorld = transformUtils.concatenateTransforms([tagToCamera.GetLinearInverse(), tagToWorld])

    existing = getKinectTransformed() is not None

    pd = filterUtils.transformPolyData(pd, cameraToWorld)
    vis.updatePolyData(pd, 'kinect transformed', colorByName='rgb_colors')

    if not existing:
        om.setActiveObject(getKinectTransformed())

    vis.updateFrame(cameraToWorld, 'kinect camera frame')


lcmUtils.addSubscriber('APRIL_TAG_TO_CAMERA_LEFT', lcmbot.rigid_transform_t, onFrame)


def testSegment():

    obj = om.findObjectByName('kinect transformed')
    segmentation.runRegionGrowingPlaneSegmentation(obj.polyData, useTerrainOptions=False, doRemoveGround=False, useVoxelGrid=False)



import continuousgraspingtaskpanel

graspPanel = continuousgraspingtaskpanel.ContinuousGraspingTaskPanel(robotSystem)
graspPanel.widget.show()


def testCrop():

    obj = om.findObjectByName('table affordance')
    tableFrame = obj.getChildFrame().transform

    xlen = obj.data.dims[0]
    ylen = obj.data.dims[1]

    polyData = segmentation.cropToBounds(getPointcloudSnapshot(), tableFrame, [[-xlen/2.0, xlen/2.0], [-ylen/2.0, ylen/2.0], [0.01, 0.4]])

    obj = vis.updatePolyData(polyData, 'cropped points', color=[0,1,0], visible=True)
    obj.setProperty('Point Size', 3)

    segmentation.runRegionGrowingPlaneSegmentation(obj.polyData, useTerrainOptions=False, doRemoveGround=False, useVoxelGrid=False)



