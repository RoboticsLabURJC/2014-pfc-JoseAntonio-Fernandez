import jderobot, threading
from interfaces.Pose3DI import Pose3DI

lock = threading.Lock()

class MissionI(jderobot.Mission):

    def __init__(self):
        self.data = jderobot.MissionData()

    def getMissionData(self, current=None):
        lock.acquire()
        print("getting Mission")
        dta = self.data
        lock.release()
        return dta

    def setMissionData(self, data, current=None):
        lock.acquire()
        print("setting Mission"+ str(data.mission))
        self.data=data
        lock.release()


    def is_empty(self):

        seq = self.data.mission
        if seq == None:
            return True
        if len(seq)>0:

            return False
        else:
            return True

