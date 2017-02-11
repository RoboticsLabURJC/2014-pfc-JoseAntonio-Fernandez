import jderobot, threading
from interfaces.Pose3DI import Pose3DI

lock = threading.Lock()

class MissionI(jderobot.Mission):

    def __init__(self):
        self.data = jderobot.MissionData()

    def getMissionData(self, current=None):
        lock.acquire()
        return self.data
        lock.release()

    def setMissionData(self, data):
        lock.acquire()
        self.data=data
        lock.release()


    def equals(self, mission):
        poses = mission.getMissionData()
        pose = Pose3DI(0, 0 ,0 ,0 ,0 ,0 ,0, 0)
        poseaux = Pose3DI(0, 0, 0, 0, 0, 0, 0, 0)
        pose.setPose3DData(poses(0))
        i=1
        for i in range(len(poses)):
            poseaux.setPose3DData(poses(i))
            if not(pose.equals(poseaux)):
                return False
        return True
