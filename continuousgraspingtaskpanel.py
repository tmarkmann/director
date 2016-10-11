from director import transformUtils
from director import lcmUtils
from director import objectmodel as om
from director import visualization as vis
from director import applogic as app
from director.debugVis import DebugData
from director import ikplanner
from director.ikparameters import IkParameters
from director import planplayback
from director import vtkNumpy as vnp
from director import ikplanner
from director.ikparameters import IkParameters
from director.tasks.taskuserpanel import TaskUserPanel
from director.tasks.taskuserpanel import ImageBasedAffordanceFit
from director import segmentation
from director import footstepsdriverpanel
from director import playbackpanel
from director import pointpicker
import director.tasks.robottasks as rt

import box_segmentation

import os
import sys
import functools
import vtkAll as vtk
import numpy as np
import drc as lcmdrc

from PythonQt import QtCore, QtGui


class ContinuousGraspingTaskPlanner(object):

    def __init__(self, robotSystem, properties):
        self.robotSystem = robotSystem
        self.robotModel = robotSystem.robotStateModel
        self.ikPlanner = robotSystem.ikPlanner
        self.properties = properties
        self.objectPicker = pointpicker.ObjectPicker(view=robotSystem.view, callback=self.onBoxPick, getObjectsFunction=self.getBoxObjects)
        self.plans = []

    def findTable(self):
        pass


    def getStanceToBoxFrame(self, aff):

        dimensions = aff.getProperty('Dimensions')

        xOffset = -(dimensions[0]/2.0 + 0.4)
        yOffset = 0.0
        zOffset = -dimensions[2]/2.0

        stanceToBox = vtk.vtkTransform()
        stanceToBox.PostMultiply()
        stanceToBox.Translate(xOffset, yOffset, zOffset)
        return stanceToBox


    def getRobotStatePose(self):
        return self.robotSystem.robotStateJointController.q.copy()

    def getPlanningStartPose(self):
        return self.getRobotStatePose()

    def addPlan(self, plan):
        self.plans.append(plan)

    def storeLastPlan(self, planName):
        plan = self.plans[-1]
        rt._addPlanItem(plan, planName, rt.ManipulationPlanItem)

    def getRobotStanceFrame(self, pose=None):
        if pose is not None:
            robotModel = self.robotSystem.ikPlanner.getRobotModelAtPose(pose)
        else:
            robotModel = self.robotSystem.robotStateModel
        return self.robotSystem.footstepsDriver.getFeetMidPoint(robotModel)

    def getFootGroundHeight(self):
        return self.getRobotStanceFrame().GetPosition()[2]


    def createElbowPostureConstraint(self, side, minJointAngle=np.radians(10)):

        name = 'l_arm_elx' if side == 'left' else 'r_arm_elx'
        sideMirror = 1 if side == 'left' else -1
        bounds = [sideMirror*minJointAngle, np.inf*sideMirror]

        constraint = ikplanner.ik.PostureConstraint()
        constraint.joints = [name]
        constraint.jointsLowerBound = [np.min(bounds)]
        constraint.jointsUpperBound = [np.max(bounds)]
        return constraint


    def planFootstepsToBox(self, startPose=None, wait=False):
        stanceToWorld = self.computeDesiredStanceFrame()

        if wait:
            if startPose is None:
                startPose = self.getPlanningStartPose()
            request = self.robotSystem.footstepsDriver.constructFootstepPlanRequest(startPose, stanceToWorld)
            footstepPlan = self.robotSystem.footstepsDriver.sendFootstepPlanRequest(request, waitForResponse=True)
            return footstepPlan
        else:
            footstepsdriverpanel.panel.onNewWalkingGoal(stanceToWorld)


    def createMovingWholeBodyGraspConstraints(self, startPose):

        ikPlanner = self.robotSystem.ikPlanner

        aff = om.findObjectByName('box')
        dimensions = aff.getProperty('Dimensions')

        startPoseName = 'plan_start'
        endPoseName = 'plan_end'

        ikPlanner.addPose(startPose, startPoseName)

        constraintSet = ikplanner.ConstraintSet(ikPlanner, [], endPoseName, startPoseName)
        constraints = constraintSet.constraints

        params = constraintSet.ikParameters
        params.maxDegreesPerSecond = 100
        params.maxBaseMetersPerSecond = 0.3
        params.quasiStaticShrinkFactor = 0.3
        params.maxBaseRPYDegreesPerSecond = 15



        constraints.append(ikPlanner.createMovingBaseSafeLimitsConstraint())
        constraints.append(ikPlanner.createMovingBackPostureConstraint()) # createMovingBackLimitedPostureConstraint
        constraints.append(ikPlanner.createBaseGroundHeightConstraint(self.getFootGroundHeight(), [0.65, np.inf]))
        constraints.extend(ikPlanner.createFixedFootConstraints(startPoseName))
        constraints.append(ikPlanner.createQuasiStaticConstraint())

        targetFrame = aff.getChildFrame()


        halfDim = [d/2.0 for d in dimensions]
        edgeBuffer = self.properties.getProperty('Edge buffer')
        palmOffset = self.properties.getProperty('Palm offset')
        coneThreshold = self.properties.getProperty('Cone threshold')
        squeezeDistance = self.properties.getProperty('Squeeze distance')

        xBound = np.clip(halfDim[0]-edgeBuffer, 0.0, np.inf)
        yBound = np.clip(halfDim[1]-squeezeDistance, 0.0, np.inf)
        zBound = np.clip(halfDim[2]-edgeBuffer, 0.0, np.inf)

        for side in ['left', 'right']:

            sideMirror = 1 if side == 'left' else -1
            graspToHandLinkFrame = ikPlanner.newPalmOffsetGraspToHandFrame(side, palmOffset)
            p = ikplanner.ik.PositionConstraint()
            p.linkName = ikPlanner.getHandLink(side)
            p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
            p.referenceFrame = targetFrame.transform
            p.lowerBound = [-xBound, yBound*sideMirror, -zBound]
            p.upperBound = [xBound, yBound*sideMirror, zBound]
            p.tspan = [1.0, 1.0]
            constraints.append(p)
            constraints.append(ikPlanner.createGazeGraspConstraint(side, targetFrame, coneThresholdDegrees=coneThreshold, targetAxis=[0.0, sideMirror*-1.0, 0.0], bodyAxis=[0.0, 1.0, 0.0]))
            constraints.append(self.createElbowPostureConstraint(side))

        lhandPalmOffset = ikPlanner.newPalmOffsetGraspToHandFrame('left', palmOffset)
        rhandPalmOffset = ikPlanner.newPalmOffsetGraspToHandFrame('right', palmOffset)
        lhandPalmOffsetToWorld = ikPlanner.newGraspToWorldFrame(startPose, 'left', lhandPalmOffset)
        rhandPalmOffsetToWorld = ikPlanner.newGraspToWorldFrame(startPose, 'right', lhandPalmOffset)
        handSeparation = np.abs(np.array(rhandPalmOffsetToWorld) - np.array(lhandPalmOffsetToWorld))

        p = ikplanner.ik.PointToPointDistanceConstraint(bodyNameA = ikPlanner.getHandLink('left'),
                                                       bodyNameB = ikPlanner.getHandLink('right'),
                                                       pointInBodyA = lhandPalmOffset.GetPosition(),
                                                       pointInBodyB = rhandPalmOffset.GetPosition,
                                                       lowerBound = [handSeparation-0.01],
                                                       upperBound = [handSeparation+0.01])
        constraints.append(p)


        #p = ikplanner.ik.PostureConstraint()
        #p.joints = [ikPlanner.backJoints[1]]
        #p.jointsLowerBound = [np.radians(12)]
        #p.jointsUpperBound = [np.inf]
        #constraints.append(p)

        return constraintSet


    def createWholeBodyGraspConstraints(self, startPose):

        ikPlanner = self.robotSystem.ikPlanner

        aff = om.findObjectByName('box')
        dimensions = aff.getProperty('Dimensions')

        startPoseName = 'plan_start'
        endPoseName = 'plan_end'

        ikPlanner.addPose(startPose, startPoseName)

        constraintSet = ikplanner.ConstraintSet(ikPlanner, [], endPoseName, startPoseName)
        constraints = constraintSet.constraints

        params = constraintSet.ikParameters
        params.maxDegreesPerSecond = self.properties.getProperty('Max degrees per second') # 100
        params.maxBaseMetersPerSecond = self.properties.getProperty('Max base meters per second') #0.3
        params.quasiStaticShrinkFactor = 0.3
        params.maxBaseRPYDegreesPerSecond = 15

        armPoseName = 'arm_pose'
        armPose = startPose.copy()
        armPose = ikPlanner.getMergedPostureFromDatabase(armPose, 'General', 'arm up pregrasp', side='left')
        armPose = ikPlanner.getMergedPostureFromDatabase(armPose, 'General', 'arm up pregrasp', side='right')
        ikPlanner.addPose(armPose, armPoseName)

        constraints.append(ikPlanner.createMovingBaseSafeLimitsConstraint())
        constraints.append(ikPlanner.createMovingBackPostureConstraint()) # createMovingBackLimitedPostureConstraint
        constraints.append(ikPlanner.createBaseGroundHeightConstraint(self.getFootGroundHeight(), [0.65, np.inf]))
        #constraints.append(ikPlanner.createLockedLeftArmPostureConstraint(armPoseName))
        #constraints.append(ikPlanner.createLockedRightArmPostureConstraint(armPoseName))
        constraints.extend(ikPlanner.createFixedFootConstraints(startPoseName))
        constraints.append(ikPlanner.createQuasiStaticConstraint())

        tq = ikplanner.ik.GravityCompensationTorqueConstraint()
        tq.joints = [ikPlanner.backJoints[1]]
        maxBackYTorque = 290
        tq.torquesLowerBound = -np.array([maxBackYTorque])
        tq.torquesUpperBound = np.array([maxBackYTorque])
        constraints.append(tq)


        targetFrame = aff.getChildFrame()

        halfDim = [d/2.0 for d in dimensions]
        edgeBuffer = self.properties.getProperty('Edge buffer')
        palmOffset = self.properties.getProperty('Palm offset')
        coneThreshold = self.properties.getProperty('Cone threshold')
        squeezeDistance = self.properties.getProperty('Squeeze distance')

        xBound = np.clip(halfDim[0]-edgeBuffer, 0.0, np.inf)
        yBound = np.clip(halfDim[1]-squeezeDistance, 0.0, np.inf)
        zBound = np.clip(halfDim[2]-edgeBuffer, 0.0, np.inf)

        for side in ['left', 'right']:

            sideMirror = 1 if side == 'left' else -1
            graspToHandLinkFrame = ikPlanner.newPalmOffsetGraspToHandFrame(side, palmOffset)
            p = ikplanner.ik.PositionConstraint()
            p.linkName = ikPlanner.getHandLink(side)
            p.pointInLink = np.array(graspToHandLinkFrame.GetPosition())
            p.referenceFrame = targetFrame.transform
            p.lowerBound = [-xBound, yBound*sideMirror, -zBound]
            p.upperBound = [xBound, yBound*sideMirror, zBound]
            constraints.append(p)

            constraints.append(ikPlanner.createGazeGraspConstraint(side, targetFrame, coneThresholdDegrees=coneThreshold, targetAxis=[0.0, sideMirror*-1.0, 0.0], bodyAxis=[0.0, 1.0, 0.0]))
            constraints.append(self.createElbowPostureConstraint(side))


        #p = ikplanner.ik.PostureConstraint()
        #p.joints = [ikPlanner.backJoints[1]]
        #p.jointsLowerBound = [np.radians(12)]
        #p.jointsUpperBound = [np.inf]
        #constraints.append(p)

        return constraintSet


    def planEndPose(self, startPose=None):
        if startPose is None:
            startPose = self.getPlanningStartPose()

        constraintSet = self.createWholeBodyGraspConstraints(startPose)
        endPose, info = constraintSet.runIk()
        self.showPose(endPose)
        return endPose

    def planSquat(self):
        startPose = self.getPlanningStartPose()
        constraintSet = self.createWholeBodyGraspConstraints(startPose)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.planEndPoseGoal()

        self.addPlan(plan)

    def planLift(self):

        startPose = self.getPlanningStartPose()
        constraintSet = self.createMovingWholeBodyGraspConstraints(startPose)
        endPose, info = constraintSet.runIk()
        plan = constraintSet.runIkTraj()

        self.addPlan(plan)

    def planStand(self):
        rt.PlanStandPosture().run()
        plan = om.findObjectByName('stand pose plan').plan
        self.addPlan(plan)

    def showPose(self, pose):
        msg = self.robotSystem.manipPlanner.createPlanMessageFromPose(pose)
        self.robotSystem.manipPlanner.onManipPlan(msg)

    def computeDesiredStanceFrame(self):
        aff = om.findObjectByName('box')
        stanceToBox = self.getStanceToBoxFrame(aff)
        boxToWorld = transformUtils.copyFrame(aff.getChildFrame().transform)
        stanceToWorld = transformUtils.concatenateTransforms([stanceToBox, boxToWorld])
        return stanceToWorld

    def computeDesiredStancePose(self):
        stanceFrame = self.computeDesiredStanceFrame()
        pos, rpy = stanceFrame.GetPosition(), transformUtils.rollPitchYawFromTransform(stanceFrame)
        q = self.robotSystem.robotStateJointController.getPose('q_nom').copy()
        q[:2] = pos[:2]
        q[5] = rpy[2]
        return q

    def getPointcloudSnapshot(self):
        return segmentation.getCurrentScanBundle()

    def getBoxObjects(self):
        return [obj for obj in om.getObjects() if obj.getProperty('Name') == 'box']

    def findBoxes(self):
        polyData = self.getPointcloudSnapshot()
        box_segmentation.fitBoxes(polyData)
        self.reorientBoxes()
        self.objectPicker.start()

    def refitBox(self):

        # get existing affordance
        assert len(self.getBoxObjects()) == 1
        aff = om.findObjectByName('box')

        # fit new affordances
        polyData = self.getPointcloudSnapshot()
        box_segmentation.fitBoxes(polyData)
        om.removeFromObjectModel(om.findObjectByName('segmentation'))
        self.reorientBoxes()

        # find closest affordance to original affordance
        def computeDist(box1, box2):
            pos = [np.array(box.getChildFrame().transform.GetPosition()) for box in [box1, box2]]
            return np.linalg.norm(pos[0] - pos[1])

        boxes = self.getBoxObjects()
        boxes.remove(aff)

        assert len(boxes)
        dists = [computeDist(aff, box) for box in boxes]

        closestIndex = np.argmin(dists)
        closestDist = dists[closestIndex]
        closestBox = boxes[closestIndex]
        boxes.remove(closestBox)

        for box in boxes:
            om.removeFromObjectModel(box)

        # keep either the original affordance or the refit affordance
        if closestDist < 0.5:
            om.removeFromObjectModel(aff)
        else:
            om.removeFromObjectModel(closestBox)
            raise Exception('affordance refit failed')

        assert len(self.getBoxObjects()) == 1


    def reorientBoxes(self):
        stanceFrame = self.getRobotStanceFrame()
        referenceAxis = transformUtils.getAxesFromTransform(stanceFrame)[0]
        box_segmentation.reorientBlocks(self.getBoxObjects(), referenceAxis)

    def onBoxPick(self, objs):
        boxes = self.getBoxObjects()
        for aff in boxes:
            if aff not in objs:
                om.removeFromObjectModel(aff)

        self.addBoxMovedCallback()
        om.removeFromObjectModel(om.findObjectByName('segmentation'))

    def spawnBox(self):

        dimensions = [0.3, 0.3, 0.3]

        pose = transformUtils.poseFromTransform(vtk.vtkTransform())
        desc = dict(classname='BoxAffordanceItem', Name='box', Dimensions=dimensions, pose=pose)
        aff = self.robotSystem.affordanceManager.newAffordanceFromDescription(desc)

        stanceToWorld = self.getRobotStanceFrame()

        boxToStance = self.getStanceToBoxFrame(aff).GetLinearInverse()
        #boxToStance = transformUtils.frameFromPositionAndRPY([1.5, 0.4, dimensions[2]/2.0], [0.0, 0.0, 20])

        boxToWorld = transformUtils.concatenateTransforms([boxToStance, stanceToWorld])

        aff.getChildFrame().copyFrame(boxToWorld)
        self.addBoxMovedCallback()

        return aff

    def addBoxMovedCallback(self):
        aff = om.findObjectByName('box')
        aff.getChildFrame().connectFrameModified(self.onBoxMoved)

    def onBoxMoved(self, boxFrame):

        if self.properties.getProperty('Replan footsteps'):
            self.planFootstepsToBox()

        if self.properties.getProperty('Replan grasp from current'):
            self.planEndPose()

        if self.properties.getProperty('Replan grasp from desired'):
            desiredStancePose = self.computeDesiredStancePose()
            self.planEndPose(desiredStancePose)

    def setFastWalkingProperties(self):
        self.robotSystem.footstepsDriver.params.setProperty('Drake Swing Speed', 3.0)
        self.robotSystem.footstepsDriver.params.setProperty('Drake Min Hold Time', 0.1)

    def setMediumFastWalkingProperties(self):
        self.robotSystem.footstepsDriver.params.setProperty('Drake Swing Speed', 2.0)
        self.robotSystem.footstepsDriver.params.setProperty('Drake Min Hold Time', 0.4)

    def planWalking(self):
        startPose = self.getPlanningStartPose()
        footstepPlan = self.robotSystem.footstepsDriver.lastFootstepPlan
        assert footstepPlan
        self.robotSystem.footstepsDriver.sendWalkingPlanRequest(footstepPlan, startPose)

    def getLastPlanEndPose(self):
        plan = self.robotSystem.manipPlanner.lastManipPlan
        planTimes, poses = self.robotSystem.planPlayback.getPlanPoses(plan)
        planTime = self.robotSystem.planPlayback.getPlanElapsedTime(plan)
        endPose = poses[-1]
        return endPose

    def planWalkToEndPose(self, endPose=None):

        if endPose is None:
            #endPose = self.getLastPlanEndPose()
            desiredStancePose = self.computeDesiredStancePose()
            endPose = self.planEndPose(desiredStancePose)

        startPose = self.getPlanningStartPose()
        stanceToWorld = self.getRobotStanceFrame(endPose)
        footstepPlan = self.planFootstepsToBox(startPose, wait=True)
        assert footstepPlan
        self.robotSystem.footstepsDriver.sendWalkingPlanRequest(footstepPlan, startPose)



class ImageFitter(ImageBasedAffordanceFit):

    def __init__(self, planner):
        ImageBasedAffordanceFit.__init__(self, numberOfPoints=1)
        self.planner = planner

    def getImageChannel(self):
        return 'CAMERA_LEFT'

    def fit(self, polyData, points):
        pass


class ContinuousGraspingTaskPanel(TaskUserPanel):

    def __init__(self, robotSystem):

        TaskUserPanel.__init__(self, windowTitle='Continuous Grasping Task')

        self.planner = ContinuousGraspingTaskPlanner(robotSystem, self.params)
        self.fitter = ImageFitter(self.planner)
        self.initImageView(self.fitter.imageView)

        self.addDefaultProperties()
        self.addButtons()
        self.addTasks()


    def addButtons(self):

        self.addManualButton('spawn box', self.spawnBox)
        self.addManualButton('find box', self.planner.findBoxes)
        self.addManualButton('refit boxes', self.planner.refitBox)
        self.addManualSpacer()
        self.addManualButton('plan footsteps', self.planner.planFootstepsToBox)
        self.addManualButton('plan walking', self.planner.planWalking)
        self.addManualButton('plan walking end pose', self.planner.planWalkToEndPose)

        self.addManualSpacer()
        self.addManualButton('plan squat', self.planner.planSquat)
        self.addManualButton('plan lift', self.planner.planSquat)
        self.addManualSpacer()
        self.addManualButton('commit', self.commitCurrentPlan)

    def spawnBox(self):
        aff = self.planner.spawnBox()

    def addDefaultProperties(self):
        self.params.addProperty('Replan footsteps', False)
        self.params.addProperty('Replan grasp from current', False)
        self.params.addProperty('Replan grasp from desired', False)

        self.params.addProperty('Edge buffer', 0.10, attributes=om.PropertyAttributes(minimum=0.0, maximum=1.0, decimals=2, singleStep=0.01))
        self.params.addProperty('Palm offset', 0.06, attributes=om.PropertyAttributes(minimum=-1.0, maximum=1.0, decimals=2, singleStep=0.01))
        self.params.addProperty('Cone threshold', 30, attributes=om.PropertyAttributes(minimum=0, maximum=90, singleStep=5))
        self.params.addProperty('Squeeze distance', 0.03, attributes=om.PropertyAttributes(minimum=-1.0, maximum=1.0, decimals=2, singleStep=0.01))

        self.params.addProperty('Max base meters per second', 0.3, attributes=om.PropertyAttributes(minimum=0.0, maximum=1.0, decimals=2, singleStep=0.01))
        self.params.addProperty('Max degrees per second', 100, attributes=om.PropertyAttributes(minimum=0, maximum=200, singleStep=10))

    def commitCurrentPlan(self):
        playbackpanel.panel.executePlan(visOnly=True)
        self.planner.robotSystem.footstepsDriver.clearFootstepPlan()


    def onPropertyChanged(self, propertySet, propertyName):
        pass

    def addTasks(self):

        # some helpers
        self.folder = None
        def addTask(task, parent=None):
            parent = parent or self.folder
            self.taskTree.onAddTask(task, copy=False, parent=parent)
        def addFunc(func, name, parent=None):
            addTask(rt.CallbackTask(callback=func, name=name), parent=parent)
        def addFolder(name, parent=None):
            self.folder = self.taskTree.addGroup(name, parent=parent)
            return self.folder


        def addManipulation(func, name, parent=None):
            planName = name + ' manip plan'
            folder = addFolder(name, parent=parent)
            addFunc(func, name='plan motion', parent=folder)
            addFunc(functools.partial(self.planner.storeLastPlan, planName), name='store last plan', parent=folder)
            addTask(rt.CheckPlanInfo(name='check manip plan info'), parent=folder)
            addTask(rt.UserPromptTask(name='approve plan', message='Please approve manip plan.'), parent=folder)
            addTask(rt.CommitManipulationPlan(name='execute manip plan', planName=planName), parent=folder)
            addTask(rt.WaitForManipulationPlanExecution(name='wait for manip execution'), parent=folder)
            #addTask(rt.UserPromptTask(name='Confirm execution has finished', message='Continue when plan finishes.'), parent=folder)


        ###############
        # add the tasks

        # prep
        prep = addFolder('prep')
        closeHands = addFolder('close hands', prep)
        addTask(rt.CloseHand(name='close left hand', side='Left'), parent=closeHands)
        addTask(rt.CloseHand(name='close right hand', side='Right'), parent=closeHands)

        # fit
        fit = addFolder('fit')
        addFunc(self.planner.findBoxes, name='find boxes', parent=fit)
        addTask(rt.UserPromptTask(name='select box affordance', message='Please select the box affordance.'), parent=fit)
        addTask(rt.FindAffordance(name='check box affordance', affordanceName='box'), parent=fit)

        # walk
        walk = self.taskTree.addGroup('approach')
        addFunc(self.planner.planFootstepsToBox, 'plan walk to box', parent=walk)
        addTask(rt.UserPromptTask(name='approve footsteps', message='Please approve footstep plan.'), parent=walk)
        addTask(rt.CommitFootstepPlan(name='walk to box'), parent=walk)
        addTask(rt.SetNeckPitch(name='set neck position', angle=45), parent=walk)
        addTask(rt.WaitForWalkExecution(name='wait for walking'), parent=walk)

        # fit
        refit = addFolder('refit')
        addFunc(self.planner.refitBox, name='refit box', parent=refit)
        addTask(rt.FindAffordance(name='check box affordance', affordanceName='box'), parent=refit)

        # grab
        addManipulation(self.planner.planSquat, 'grab box')
        addManipulation(self.planner.planStand, 'lift box')



