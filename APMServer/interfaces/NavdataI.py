import jderobot, threading

lock = threading.Lock()

class NavdataI(jderobot.Navdata):

    def __init__(self):
        self.data = jderobot.NavdataData()

    def getNavdata(self, current=None):
        lock.acquire()
        return self.data
        lock.release()

    def setNavdata(self, data):
        lock.acquire()
        self.data=data
        lock.release()