import jderobot, threading, traceback, Ice

lock = threading.Lock()

class MissionI(jderobot.Mission):

    def __init__(self, ic, prefix):
        try:
            base = ic.propertyToProxy(prefix+".Proxy")
            self.proxy = jderobot.MissionPrx.checkedCast(base)
            prop = ic.getProperties()

            if not self.proxy:
                print ('Interface ' + prefix + ' not configured')

        except Ice.ConnectionRefusedException:
            print(prefix + ': connection refused')

        except:
            traceback.print_exc()
            exit(-1)

    def getMissionData(self, current=None):
        if self.hasproxy():
            return self.proxy.getMissionData()

    def setMissionData(self, data, current=None):
        if self.hasproxy():
            return self.proxy.setMissionData(data)

    def is_empty(self):
        if self.data == None:
            return True
        if self.data.len>0:
            return False
        else:
            return True

    def hasproxy (self):
        return hasattr(self,"proxy") and self.proxy

    class PoseSequence(jderobot.Mission):
        None

