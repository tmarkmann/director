from director.lcmlogplayer import LcmLogPlayer, LcmLogPlayerGui
from director import lcmspy
from director import lcmUtils
import drake as lcmdrake


def getDrakeSimTimeForEvent(event):
    if not event.channel.startswith('DRAKE_VIEWER'):
        return None
    msg = lcmspy.decodeMessage(event.data)
    if hasattr(msg, 'timestamp'):
        return msg.timestamp*1000
    else:
        print 'failed to get time for message on', event.channel


def initLcmSpy():
    lcmspy.loadMessageTypes(lcmspy.messageTypes, lcmdrake)


def initDrakeTimeDisplay():

    def onViewerDraw(msg):
        t = msg.timestamp*1e-3
        vis.updateText('sim time: %.3f' % t, 'sim time')

    lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmdrake.lcmt_viewer_draw, onViewerDraw)


initLcmSpy()
initDrakeTimeDisplay()

#filename = 'kuka_sim.lcmlog'


filename = '/home/pat/Desktop/logs/sim-funnel-grasp.lcmlog'

print 'reading log:', filename

logPlayer = LcmLogPlayer()
logPlayer.playbackFactor = 1.0
logPlayer.readLog(filename, eventTimeFunction=getDrakeSimTimeForEvent)
logPlayer.skipToTime(0.0)
logPlayerGui = LcmLogPlayerGui(logPlayer)
