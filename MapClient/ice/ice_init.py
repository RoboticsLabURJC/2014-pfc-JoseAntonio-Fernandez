import threading,time,sys,traceback
import jderobot,Ice
from MapClient.classes import MissionI

def send_mission(mission):
    '''
    Open a Mission client to send a Mission with waypoints
    :return:  mone
    '''
    status = 0
    ic = None

    try:
        print("Recibido la mision")
        print(mission)
        print("send")
        ic = Ice.initialize(sys.argv)
        base = ic.stringToProxy("mision:default -p 9990")
        prx = jderobot.MissionPrx.checkedCast(base)
        prx.mission=(mission.getMissionData())
        print(str(mission) + "\n mandado")
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