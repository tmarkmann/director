

from director import lcmUtils
from director.shallowCopy import shallowCopy
import vicon as lcmvicon

class DrawVicon(object):

    def __init__(self, channel):
        self.channel = channel
        self.subscriber = None
        self.lastMessage = None
        self.unitConversion = 0.001
        self.models = {}
        self.markerGeometry = None
        self.clear()
        self.initSubscriber()

    def initSubscriber(self):
        self.subscriber = lcmUtils.addSubscriber(self.channel, lcmvicon.vicon_t, self.onMessage)
        self.subscriber.setSpeedLimit(10)

    def removeSubscriber(self):
        if not self.subscriber:
            return
        lcmUtils.removeSubscriber(self.subscriber)
        self.subscriber = None

    def getRootFolder(self):
        folder = om.getOrCreateContainer(self.channel)
        return folder

    def onMessage(self, msg):
        self.lastMessage = msg
        self.drawModels(msg)

    def getMarkerGeometry(self):

        if self.markerGeometry is None:
            d = DebugData()
            d.addSphere(np.zeros(3), radius=0.007, resolution=12)
            self.markerGeometry = shallowCopy(d.getPolyData())

        return self.markerGeometry

    def clear(self):
        om.removeFromObjectModel(self.getRootFolder())

    def drawModels(self, msg):

        tNow = time()

        for model in msg.models:
            self.drawModel(model)

        elapsed = time() - tNow
        #print 'rate:', 1/elapsed

    def createMarkerObjects(self, numberOfMarkers, modelFolder, modelName, modelColor):

        geom = self.getMarkerGeometry()

        def makeMarker(i):
            obj = vis.showPolyData(shallowCopy(geom), modelName + ' marker %d' % i, color=modelColor, parent=modelFolder)
            vis.addChildFrame(obj)
            return obj

        return [makeMarker(i) for i in xrange(numberOfMarkers)]


    def drawModel(self, model):


        modelFolder = om.getOrCreateContainer(model.name, parentObj=self.getRootFolder())
        markerFolder = om.getOrCreateContainer('markers', parentObj=modelFolder)
        modelName = model.name

        markerObjects = markerFolder.children()

        if len(markerObjects) != model.nummarkers:
            for obj in markerObjects:
                om.removeFromObjectModel(obj)

            modelColor = segmentation.getRandomColor()
            markerObjects = self.createMarkerObjects(model.nummarkers, markerFolder, modelName, modelColor)
            self.models[modelName] = markerObjects

        if len(markerObjects):
            modelColor = markerObjects[0].getProperty('Color')

        for i, marker in enumerate(model.markers):
            xyz = np.array(marker.xyz)*self.unitConversion
            markerFrame = vtk.vtkTransform()
            markerFrame.Translate(xyz)
            markerObjects[i].getChildFrame().copyFrame(markerFrame)

        computeModelEdges = False
        if computeModelEdges:
            d = DebugData()
            for m1 in model.markers:
                xyz = np.array(m1.xyz)*self.unitConversion
                for m2 in model.markers:
                    xyz2 = np.array(m2.xyz)*self.unitConversion
                    d.addLine(xyz, xyz2)
            edges = shallowCopy(d.getPolyData())
            vis.updatePolyData(edges, modelName + ' edges', color=modelColor, parent=modelFolder)

        computeModelFrame = False
        if computeModelFrame:
            pos = np.array(model.segments[0].T)*self.unitConversion
            rpy = np.array(model.segments[0].A)
            modelFrame = transformUtils.frameFromPositionAndRPY(pos, np.degrees(rpy))
            vis.updateFrame(modelFrame, modelName + ' frame', parent=modelFolder, scale=0.1)



drawVicon = DrawVicon('drc_vicon')
