import threading,time,sys,traceback
import jderobot,Ice
from MapClient.classes import Pose3DI

def sendWP(pose):
    '''
    Open a Pose3D client to recieve Pose3D with a waypoint
    :return:  mone
    '''
    status = 0
    ic = None

    try:
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("WP_pose3d:default -p 9994")
        prx = jderobot.Pose3DPrx.checkedCast(base)
        prx.setPose3DData(pose)
        print(str(pose) + "\n mandado")
    except:
        traceback.print_exc()
        status = 1

    if ic:
        # Clean up
        try:
            ic.destroy()
        except:
            traceback.print_exc()
            status = 1

    sys.exit(status)