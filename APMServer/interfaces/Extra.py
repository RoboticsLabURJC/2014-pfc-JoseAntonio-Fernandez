__author__ = 'AeroCano'

import jderobot, time, threading

lockLand = threading.Lock()
lockTakeOff = threading.Lock()

class ExtraI(jderobot.ArDroneExtra):

    def __init__(self):

        self.landDecision = False
        self.takeOffDecision = False

    def land(self, _ctx=None):
        lockLand.acquire()
        self.landDecision = True
        lockLand.release()

        #return landDecision

    def takeoff(self, _ctx=None):
        lockTakeOff.acquire()
        self.takeOffDecision = True
        lockTakeOff.release()

        #return self.takeOffDecision

    def setLand(self,decision):

        lockLand.acquire()
        self.landDecision = decision
        lockLand.release()

    def setTakeOff(self, decision):

        lockTakeOff.acquire()
        self.takeOffDecision = decision
        lockTakeOff.release()

    def setExtraData(self, data, current=None):

        lockLand.acquire()
        self.landDecision = data.landDecision
        lockLand.release()

        lockTakeOff.acquire()
        self.takeOffDecision = data.takeOffDecision
        lockTakeOff.release()

        return 0
