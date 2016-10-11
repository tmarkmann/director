import continuousgraspingtaskpanel

graspPanel = continuousgraspingtaskpanel.ContinuousGraspingTaskPanel(robotSystem)
graspPanel.widget.show()

#graspPanel.spawnBox()

for model in [robotStateModel, teleopRobotModel, playbackRobotModel]:
    roboturdf.setRobotiqJointsToClosedHand(model)

p = graspPanel
#p.planner.setFastWalkingProperties()
