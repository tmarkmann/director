from director import depthscanner
from director import ioUtils
from director import lcmframe
from director import lcmUtils
from director import cameraview
from director import vtkAll as vtk
from director.shallowCopy import shallowCopy
from director.utime import getUtime
from director.filterUtils import flipImage
import numpy as np


def test():
    depthScanner = depthscanner.DepthScanner(view)
    depthScanner.update()
    depthScanner.imageView.view.show()


def initImageManager():

    imageManager = cameraview.ImageManager()
    cameraview.imageManager = imageManager
    return imageManager


def loadAndFlipImage(filename):
    image = ioUtils.readImage(filename)
    return flipImage(image)


def testReadAndPublish(channel, imageManager):
    im = loadAndFlipImage('color_buffer.vti')
    print im.GetDimensions()
    print im.GetScalarTypeAsString()

    imDepth = loadAndFlipImage('depth_image_uint16.vti')
    print imDepth.GetDimensions()
    print imDepth.GetScalarTypeAsString()

    imageManager.queue.publishRGBDImagesMessage(channel, im, imDepth, getUtime())


def publishDepthScanner(channel, imageManager, depthScanner, utime):
    colorImage = depthScanner.getColorBufferImage()
    depthImage = depthScanner.getDepthImage()

    colorImage = flipImage(colorImage)
    depthImage = flipImage(depthImage)

    #imageManager.queue.publishRGBImageMessage('TEST_IMAGE', colorImage, utime)
    imageManager.queue.publishRGBDImagesMessage(channel, colorImage, depthImage, utime)


def publishCameraPose(camera, channel, utime):

    lookAt = transformUtils.getLookAtTransform(camera.GetFocalPoint(), camera.GetPosition(), camera.GetViewUp())
    xaxis, yaxis, zaxis = transformUtils.getAxesFromTransform(lookAt)
    xaxis, yaxis, zaxis = -yaxis, -zaxis, xaxis
    cameraToWorld = transformUtils.getTransformFromAxesAndOrigin(xaxis, yaxis, zaxis, camera.GetPosition())


    cameraToWorldMsg = lcmframe.rigidTransformMessageFromFrame(cameraToWorld)
    cameraToWorldMsg.utime = utime
    lcmUtils.publish(channel, cameraToWorldMsg)


def syncCamera(camera):
    obj = om.findObjectByName('iiwa_link_7 geometry data')
    if not obj:
        return

    linkToWorld = obj.actor.GetUserTransform()
    if not linkToWorld:
        return

    cameraToLink = transformUtils.frameFromPositionAndRPY([0.0, 0.0, 0.3], [0, 0, 0])
    cameraToWorld = transformUtils.concatenateTransforms([cameraToLink, linkToWorld])

    origin = np.array(cameraToWorld.GetPosition())
    axes = transformUtils.getAxesFromTransform(cameraToWorld)

    camera.SetPosition(origin)
    camera.SetFocalPoint(origin+axes[2])
    camera.SetViewUp(-axes[0])



class MyDepthScanner(depthscanner.DepthScanner):

    def update(self):
        #syncCamera(view.camera())
        depthscanner.DepthScanner.update(self)
        utime = getUtime()
        publishCameraPose(view.camera(), 'OPENNI_FRAME_LEFT_TO_LOCAL', utime)
        publishDepthScanner('OPENNI_FRAME', imageManager, self, utime)


def focalLengthToViewAngle(focalLength, imageHeight):
    '''Returns a view angle in degrees that can be set on a vtkCamera'''
    return np.degrees(2.0 * np.arctan2(imageHeight/2.0, focalLength))


def viewAngleToFocalLength(viewAngle, imageHeight):
    '''Returns the focal length given a view angle in degrees from a vtkCamera'''
    return (imageHeight/2.0)/np.tan(np.radians(viewAngle/2.0))


def setCameraIntrinsics(view, principalX, principalY, focalLength):

    imageWidth = view.width
    imageHeight = view.height

    wcx = -2*(principalX - float(imageWidth)/2) / imageWidth
    wcy =  2*(principalY - float(imageHeight)/2) / imageHeight
    viewAngle = focalLengthToViewAngle(focalLength, imageHeight)

    print 'image dims:', imageWidth, imageHeight
    print 'window center:', wcx, wcy
    print 'viewAngle:', viewAngle

    camera = view.camera()
    camera.SetWindowCenter(wcx, wcy)
    camera.SetViewAngle(viewAngle)


def setCameraInstrinsicsAsus(view):
    # momap default
    #principalX = 319.5
    #principalY = 239.5
    #focalLength = 570.3422241210938


    principalX = 320.0
    principalY = 267.0
    focalLength = 528.01442863461716

    setCameraIntrinsics(view, principalX, principalY, focalLength)


def setCameraInstrinsicsBasic(view):

    imageWidth = view.width
    imageHeight = view.height

    principalX = 640/2.0
    principalY = 480/2.0

    imageHeight = 480
    viewAngle = view.camera().GetViewAngle()
    focalLength = viewAngleToFocalLength(viewAngle, imageHeight)

    setCameraIntrinsics(view, principalX, principalY, focalLength)


view.setMinimumSize(640, 480)
view.setMaximumSize(640, 480)
imageManager = initImageManager()
viewOptions.setProperty('Orientation widget', False)

depthScanner = MyDepthScanner(view)
depthScanner.update()

setCameraInstrinsicsAsus(view)
#setCameraInstrinsicsBasic(view)

