from director import mainwindowapp
from director import robotsystem
from director import applogic
from PythonQt import QtCore

def makeRobotSystem(view):
    factory = robotsystem.RobotSystemFactory()
    options = factory.getDisabledOptions()
    factory.setDependentOptions(options, usePlannerPublisher=True, useTeleop=True)
    return factory.construct(view=view, options=options)

app = mainwindowapp.MainWindowAppFactory().construct()
robotSystem = makeRobotSystem(app.view)
robotSystem.ikPlanner.planningMode = 'pydrake'

app.app.addWidgetToDock(robotSystem.teleopPanel.widget, QtCore.Qt.RightDockWidgetArea)
app.app.addWidgetToDock(robotSystem.playbackPanel.widget, QtCore.Qt.BottomDockWidgetArea)
applogic.resetCamera(viewDirection=[-1,0,0], view=app.view)


from PythonQt import QtGui
from director import lcmUtils
import drake as lcmdrake

class KukaSimInfoLabel(object):
    '''
    Displays simulation time and frequency in the status bar
    '''

    def __init__(self, statusBar):
        self.label = QtGui.QLabel('')
        statusBar.addPermanentWidget(self.label)

        self.sub = lcmUtils.addSubscriber('IIWA_STATUS', lcmdrake.lcmt_iiwa_status, self.onIiwaStatus)
        self.sub.setSpeedLimit(30)

        self.label.text = '[waiting for sim status]'

    def onIiwaStatus(self, msg):
        simTime = msg.timestamp*1e-6
        simFreq = self.sub.getMessageRate()
        self.label.text = 'Sim freq: %d hz  |  Sim time: %.2f' % (simFreq, simTime)

infoLabel = KukaSimInfoLabel(app.mainWindow.statusBar())


app.app.start()
