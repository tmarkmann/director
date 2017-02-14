import os
import subprocess

import pydrake
installDir = os.path.join(pydrake.getDrakePath(), '../build/install')

def checkOutput(outputStr):
    lines = outputStr.split('\n')
    for line in lines:
        if line.startswith('File:'):
            print line
        if 'TLS' in line:
            print line

commands = ['find %s -name \*.so | xargs readelf -l' % installDir,
            'find %s -name \*.so | xargs readelf --dynamic' % installDir]

#for command in commands:
#    print 'COMMAND:', command
#    outputStr = subprocess.check_output(command, shell=True)
#    checkOutput(outputStr)




print '----------------------------------------------------------'

command = 'find %s -name \*.so' % installDir
print 'COMMAND:', command
outputStr = subprocess.check_output(command, shell=True)
soFiles = [s for s in outputStr.split() if os.path.isfile(s)]

for filename in list(soFiles):

    command = 'ldd %s' % filename
    try:
        outputStr = subprocess.check_output(command, shell=True)
        soFiles.extend([s for s in outputStr.split() if os.path.isfile(s)])
    except:
        print 'exception'
        pass

soFiles = sorted(list(set(soFiles)))

print 'checking readelf -l'
for filename in soFiles:
    print filename
    try:
        print subprocess.check_output('readelf -l %s | grep TLS' % filename, shell=True)
    except:
        continue

print '----------------------------------------------------------'
print 'checking readelf --dynamics'
for filename in soFiles:
    print filename
    try:
        print subprocess.check_output('readelf --dynamic %s | grep TLS' % filename, shell=True)
    except:
        continue



from director import mainwindowapp
from director import robotsystem
from director import applogic
from PythonQt import QtCore

def makeRobotSystem(view):
    factory = robotsystem.ComponentFactory()
    factory.register(robotsystem.RobotSystemFactory)
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)
    return factory.construct(view=view, options=options)

app = mainwindowapp.construct()


robotSystem = makeRobotSystem(app.view)

app.app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea)

# use pydrake ik backend
ikPlanner = robotSystem.ikPlanner
ikPlanner.planningMode = 'pydrake'
ikPlanner.plannerPub._setupLocalServer()

applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)
app.app.start()
