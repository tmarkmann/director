from director.debugVis import DebugData
from director import filterUtils
from director import lcmframe
from director import lcmUtils
from director import visualization as vis
from director import vtkAll as vtk
import bot_core as lcmbotcore
import numpy as np


class GoalFramePublisher(object):

    def __init__(self, publishChannel='GOAL_FRAME'):
        self.publishChannel = publishChannel

    def addFrameWidget(self, name='goal frame'):

        # show a new frame
        goalFrame = vis.showFrame(vtk.vtkTransform(), name)

        # initialize the widget
        self.initWidgetAppearance(goalFrame)

        # connect a callback to the frame modified signal
        goalFrame.connectFrameModified(self.onGoalFrameModified)
        self.onGoalFrameModified(goalFrame)

    def onGoalFrameModified(self, frame):
        # publish the frame as a bot_core.rigid_transform_t message
        msg = lcmframe.rigidTransformMessageFromFrame(frame.transform)
        lcmUtils.publish(self.publishChannel, msg)

    def initWidgetAppearance(self, frameObj):

        # set default size
        frameObj.setProperty('Scale', 0.25)

        # widget is active by default
        frameObj.setProperty('Edit', True)

        # disable translation in z and rotation in x,y
        rep = frameObj.widget.GetRepresentation()
        rep.SetTranslateAxisEnabled(2, False)
        rep.SetRotateAxisEnabled(0, False)
        rep.SetRotateAxisEnabled(1, False)
        frameObj.widget.HandleRotationEnabledOff()


class DummyPlanner(object):

    def __init__(self, channel='GOAL_FRAME'):
        self.sub = lcmUtils.addSubscriber(channel, lcmbotcore.rigid_transform_t, self.onMessage)

    def onMessage(self, msg):

        # make a bunch of random points in the goal frame
        t = lcmframe.frameFromRigidTransformMessage(msg)
        pts = np.random.rand(30,3)*0.25+[1.0,0.0,0.0]
        pts = np.array([t.TransformPoint(p) for p in pts])

        # pack the points in a flat list and publish
        msg = lcmbotcore.viewer_geometry_data_t()
        msg.float_data = pts.flatten()
        msg.num_float_data = len(msg.float_data)
        lcmUtils.publish('PLAN_RESPONSE', msg)


class PlanListener(object):

    def __init__(self, responseChannel='PLAN_RESPONSE'):
        self.sub = lcmUtils.addSubscriber(responseChannel, lcmbotcore.viewer_geometry_data_t, self.onMessage)

    def onMessage(self, msg):

        # unpack the points and draw them as spheres
        pts = np.array(msg.float_data).reshape(msg.num_float_data/3, 3)
        d = DebugData()
        for p in pts:
            d.addSphere(p, radius=0.03)

        # update the geometry data
        geometryName = 'plan response'
        vis.updatePolyData(d.getPolyData(), geometryName, color=[0,1,0])


# make a goal frame publisher
goalFramePublisher = GoalFramePublisher()
goalFramePublisher.addFrameWidget()

# make a dummy planner that listens for the goal frame message
planner = DummyPlanner()

# make a listener that draws the plan published by the DummyPlanner
planListener = PlanListener()
