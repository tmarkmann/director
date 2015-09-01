from ddapp import transformUtils
from ddapp import lcmUtils
from ddapp import objectmodel as om
from ddapp import visualization as vis
from ddapp import applogic as app
from ddapp.debugVis import DebugData
from ddapp import ikplanner
from ddapp.ikparameters import IkParameters
from ddapp import planplayback
from ddapp import vtkNumpy as vnp

from ddapp.timercallback import TimerCallback
from ddapp.asynctaskqueue import AsyncTaskQueue
from ddapp.footstepsdriver import FootstepRequestGenerator
from ddapp.tasks.taskuserpanel import TaskUserPanel
from ddapp.tasks.taskuserpanel import ImageBasedAffordanceFit

import os
import sys
import time
import vtkAll as vtk
import numpy as np
import drc as lcmdrc

from PythonQt import QtCore, QtGui


class ContinuousWalkingTaskPlanner(object):

    def __init__(self, robotSystem):
        self.robotSystem = robotSystem
        self.footstepPlanner = robotSystem.footstepsDriver
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner

        self.autoCommit = True
        self.nextFootstepPlan = None
        self.nextFootstepPlanPose = None
        self.lastFootstepPlanMessage = None
        self.leftFootInContact = True
        self.rightFootInContact = True
        self.planningTaskQueue = AsyncTaskQueue()

        lcmUtils.addSubscriber('WALKING_CONTROLLER_PLAN_REQUEST', lcmdrc.walking_plan_request_t, self.onCommittedFootstepPlan)
        lcmUtils.addSubscriber('WALKING_CONTROLLER_QUEUED_PLAN_REQUEST', lcmdrc.walking_plan_request_t, self.onCommittedFootstepPlan)
        sub = lcmUtils.addSubscriber('FOOT_CONTACT_ESTIMATE', lcmdrc.foot_contact_estimate_t, self.onFootContact)
        sub.setSpeedLimit(60)


    def getControllerStatus(self):
        return self.robotSystem.atlasDriver.lastControllerStatusMessage

    def sideToFootLinkName(self, side):
        return {'left':self.ikPlanner.leftFootLink, 'right':self.ikPlanner.rightFootLink}[side]


    def getDistanceToFrame(self, frameA, frameB):
        return np.linalg.norm(np.array(frameA.GetPosition()) - np.array(frameB.GetPosition()))


    def findFootstepIndex(self, side):

        stepFrames = self.getFootstepFrames(self.lastFootstepPlanMessage)

        linkName = self.sideToFootLinkName(side)
        startPose = self.robotSystem.robotStateJointController.q.copy()
        footFrame = self.robotSystem.ikPlanner.getLinkFrameAtPose(linkName, startPose)

        dists = [self.getDistanceToFrame(footFrame, stepFrame) for stepFrame in stepFrames]

        #print 'footstep dists for foot', side

        for i, step in enumerate(self.lastFootstepPlanMessage.footsteps):
            if side == 'right' and not step.is_right_foot:
                dists[i] = 1e6
            elif side == 'left' and step.is_right_foot:
                dists[i] = 1e6

            #print '%2d %s: %.3f' % (i, 'right' if step.is_right_foot else 'left', dists[i])

        return np.argmin(dists)


    def readyToPlanContinuousWalking(self):

        if not self.getControllerStatus():
            print 'no controller status message'
            return False
        if self.getControllerStatus().plan_type != lcmdrc.plan_status_t.WALKING:
            print 'not walking'
            return False
        if not self.lastFootstepPlanMessage:
            print 'active footstep plan is missing'
            return False
        if not (self.leftFootInContact or self.rightFootInContact):
            print 'at least one foot must be in contact'
            return False

        return True


    def isLeftStep(self, footstepPlan, stepIndex):
        return not self.isRightStep(footstepPlan, stepIndex)

    def isRightStep(self, footstepPlan, stepIndex):
        return footstepPlan.footsteps[stepIndex].is_right_foot

    def stepSide(self, footstepPlan, stepIndex):
        return 'left' if self.isLeftStep(footstepPlan, stepIndex) else 'right'

    def computeNextStepIndex(self):

        if self.leftFootInContact:
            leftStepIndex = self.findFootstepIndex('left')
        else:
            leftStepIndex = -1

        if self.rightFootInContact:
            rightStepIndex = self.findFootstepIndex('right')
        else:
            rightStepIndex = -1

        assert rightStepIndex != leftStepIndex

        print 'left step index (contact %r):' % self.leftFootInContact, leftStepIndex
        print 'right step index (contact %r):' % self.rightFootInContact, rightStepIndex

        nextStepIndex = max(leftStepIndex, rightStepIndex)

        print 'selecting next step index:', nextStepIndex

        assert nextStepIndex >= 0


        print 'next step index:', nextStepIndex
        print 'next step side:', self.stepSide(self.lastFootstepPlanMessage, nextStepIndex)

        return nextStepIndex


    def showNextStepPose(self):

        if not self.readyToPlanContinuousWalking():
            return

        nextStepIndex = self.computeNextStepIndex()
        pose = self.computeStancePoseFromFootstepPlan(self.lastFootstepPlanMessage, nextStepIndex)
        self.showPose(pose)


    def planNextSteps(self):

        self.nextFootstepPlan = None
        self.nextFootstepPlanPose = None

        if not self.readyToPlanContinuousWalking():
            return

        nextStepIndex = self.computeNextStepIndex()

        pose = self.computeStancePoseFromFootstepPlan(self.lastFootstepPlanMessage, nextStepIndex)
        if not pose:
            return

        self.showPose(pose)

        stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(self.ikPlanner.getRobotModelAtPose(pose))


        goalFrame = om.findObjectByName('joystick walking goal')

        goalFrame = goalFrame.transform if goalFrame else stanceFrame


        request = self.footstepPlanner.constructFootstepPlanRequest(pose, goalFrame)

        if self.isLeftStep(self.lastFootstepPlanMessage, nextStepIndex):
            request.params.leading_foot = lcmdrc.footstep_plan_params_t.LEAD_LEFT
        else:
            request.params.leading_foot = lcmdrc.footstep_plan_params_t.LEAD_RIGHT

        request.params.min_num_steps = 4
        request.params.max_num_steps = 6

        #request.params.max_forward_step = 0.5
        #request.params.nom_forward_step = 0.12
        #request.params.nom_step_width = 0.22
        #request.params.max_num_steps = 8

        #plan = self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=True)

        self.nextFootstepPlanPose = pose

        responseHelper = lcmUtils.MessageResponseHelper('FOOTSTEP_PLAN_RESPONSE', lcmdrc.footstep_plan_t)
        self.footstepPlanner.sendFootstepPlanRequest(request, waitForResponse=False)
        self.waitForPlanAsync(responseHelper)


    def waitForPlanAsync(self, responseHelper):

        def waitForResponse():

            tStart = time.time()
            while (time.time() - tStart) < 1.0:
                self.nextFootstepPlan = responseHelper.waitForResponse(timeout=0)
                if not self.nextFootstepPlan:
                    yield
                else:
                    break

            print 'got plan after %.5f seconds' % (time.time() - tStart)
            responseHelper.finish()

            if not self.nextFootstepPlan:
                raise Exception('timeout waiting for footstep plan')


        def checkAutoCommit():
            if self.autoCommit:
                self.commitNextFootstepPlan()

        t = self.planningTaskQueue
        t.stop()
        t.reset()
        t.addTask(waitForResponse)
        t.addTask(checkAutoCommit)
        t.start()


    def commitNextFootstepPlan(self):
        assert self.nextFootstepPlan is not None
        self.footstepPlanner.sendWalkingPlanRequest(self.nextFootstepPlan, self.nextFootstepPlanPose, req_type='queued_controller')


    def onCommittedFootstepPlan(self, msg):
        self.lastFootstepPlanMessage = msg.footstep_plan

    def onFootContact(self, msg):
        self.leftFootInContact = msg.left_contact > 0.0
        self.rightFootInContact = msg.right_contact > 0.0

    def getFootstepFrames(self, footstepPlan):

        def stepToFrame(step):
            trans = step.pos.translation
            trans = [trans.x, trans.y, trans.z]
            quat = step.pos.rotation
            quat = [quat.w, quat.x, quat.y, quat.z]
            return transformUtils.transformFromPose(trans, quat)

        return [stepToFrame(step) for step in footstepPlan.footsteps]



    def createQuasiStaticConstraint(self, leftFootEnabled=True, rightFootEnabled=True):
        qs = ik.QuasiStaticConstraint(leftFootEnabled=leftFootEnabled, rightFootEnabled=rightFootEnabled,
                                                    pelvisEnabled=False,
                                                    shrinkFactor=0.05)
        qs.leftFootLinkName = self.robotSystem.ikPlanner.leftFootLink
        qs.rightFootLinkName = self.robotSystem.ikPlanner.rightFootLink
        return qs


    def computeWeightShiftedStancePose(self, leftFootFrame, rightFootFrame, weightShiftMode):

        assert weightShiftMode in ['left', 'right', 'center']

        leftFootEnabled = weightShiftMode in ('left', 'center')
        rightFootEnabled = weightShiftMode in ('right', 'center')

        startPose = self.robotSystem.robotStateJointController.getPose('q_nom').copy()

        startPoseName = 'my_pose_start'
        endPoseName = 'my_pose_end'

        self.robotSystem.ikPlanner.addPose(startPose, startPoseName)

        constraints = []

        constraints.append(self.robotSystem.ikPlanner.createLockedBackPostureConstraint(startPoseName))
        constraints.append(self.createQuasiStaticConstraint(leftFootEnabled=leftFootEnabled, rightFootEnabled=rightFootEnabled))
        constraints.append(self.robotSystem.ikPlanner.createXYZYawMovingBasePostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedLeftArmPostureConstraint(startPoseName))
        constraints.append(self.robotSystem.ikPlanner.createLockedRightArmPostureConstraint(startPoseName))

        nullFrame = vtk.vtkTransform()
        constraints.extend(self.ikPlanner.createPositionOrientationConstraint(self.ikPlanner.rightFootLink, rightFootFrame, vtk.vtkTransform()))
        constraints.extend(self.ikPlanner.createPositionOrientationConstraint(self.ikPlanner.leftFootLink, leftFootFrame, vtk.vtkTransform()))

        constraintSet = ikplanner.ConstraintSet(self.robotSystem.ikPlanner, constraints, endPoseName, startPoseName)
        pose, info = constraintSet.runIk()
        return pose


    def computeStancePoseFromFootstepPlan(self, footstepPlan, trailingStepIndex):

        assert 0 <= trailingStepIndex < len(footstepPlan.footsteps)


        if trailingStepIndex+1 == len(self.lastFootstepPlanMessage.footsteps):
            # we should use the end stance with centered weight in this case
            print 'trailing step index is end of footstep plan, not supported yet'
            return None

        frames = self.getFootstepFrames(footstepPlan)

        trailingFootFrame = frames[trailingStepIndex]
        leadingFootFrame = frames[trailingStepIndex + 1]

        if footstepPlan.footsteps[trailingStepIndex].is_right_foot:
            leftFootFrame = leadingFootFrame
            rightFootFrame = trailingFootFrame
            weightShiftMode = 'right'
        else:
            leftFootFrame = trailingFootFrame
            rightFootFrame = leadingFootFrame
            weightShiftMode = 'left'

        return self.computeWeightShiftedStancePose(leftFootFrame, rightFootFrame, weightShiftMode)

    def showStancePose(self, footstepPlan, trailingStepIndex):
        pose = self.computeStancePoseFromFootstepPlan(footstepPlan, trailingStepIndex)
        self.showPose(pose)

    def showPose(self, pose):
        self.robotSystem.playbackRobotModel.setProperty('Visible', True)
        self.robotSystem.playbackRobotModel.setProperty('Alpha', 0.05)
        self.robotSystem.playbackJointController.setPose('show_pose', pose)


class SingleSupportDetector(object):

    def __init__(self, planner):
        self.planner = planner
        self.timer = TimerCallback()
        self.timer.callback = self.tick
        self.lastSingleSupportTime = np.nan
        self.inSingleSupport = False
        self.requiredSingleSupportTime = 0.2

    def tick(self):
        isSingleSupport = not (self.planner.rightFootInContact and self.planner.leftFootInContact)

        if not isSingleSupport:
            self.inSingleSupport = False
            if not np.isnan(self.lastSingleSupportTime):
                self.lastSingleSupportTime = np.nan
                print 'transition to single support'
        elif np.isnan(self.lastSingleSupportTime):
            self.lastSingleSupportTime = time.time()
            print 'detected single support'
        elif (time.time() - self.lastSingleSupportTime) > self.requiredSingleSupportTime:
            if not self.inSingleSupport:
                print 'setting single support'
                self.inSingleSupport = True


class AutoWalkTrigger(object):

    def __init__(self, planner):
        self.planner = planner
        self.detector = SingleSupportDetector(planner)
        self.taskQueue = AsyncTaskQueue()

    def stop(self):
        self.taskQueue.stop()
        self.taskQueue.reset()
        self.detector.timer.stop()

    def restart(self):
        self.stop()
        self.addTasks()
        self.detector.timer.start()
        self.taskQueue.start()

    def addTasks(self):
        self.taskQueue.addTask(self.waitForNextDoubleSupport)
        self.taskQueue.addTask(self.plan)
        self.taskQueue.addTask(self.addTasks)

    def waitForNextDoubleSupport(self):

        print 'wait for single support...'
        while not self.detector.inSingleSupport:
            yield

        print '...got single support.'
        print 'wait for double support...'

        while self.detector.inSingleSupport:
            yield
        print 'got double support.'


    def waitForNextSingleSupport(self):

        print 'wait for double support...'
        while self.detector.inSingleSupport:
            yield

        print '...got double support.'
        print 'wait for single support...'

        while not self.detector.inSingleSupport:
            yield
        print 'got single support.'


    def plan(self):
        print 'planning next steps now'
        self.planner.planNextSteps()


class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, planner):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.planner = planner

    def fit(self, polyData, points):
        pass


class ContinuousWalkingTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Continuous Walking Task')

        self.planner = ContinuousWalkingTaskPlanner(robotSystem)
        self.fitter = ImageFitter(self.planner)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def addButtons(self):

        self.addManualButton('plan next steps', self.planner.planNextSteps)
        self.addManualSpacer()


    def addDefaultProperties(self):
        self.params.addProperty('Auto walk', False)

    def onPropertyChanged(self, propertySet, propertyName):
        pass

    def addTasks(self):
        pass


def makeJoystickGoalFrame():
    stanceFrame = FootstepRequestGenerator.getRobotStanceFrame(robotStateModel)
    return vis.updateFrame(stanceFrame, 'joystick walking goal', scale=0.2)


makeJoystickGoalFrame()

p  = ContinuousWalkingTaskPanel(robotSystem)
#p.widget.show()
planner = p.planner

trigger = AutoWalkTrigger(planner)

