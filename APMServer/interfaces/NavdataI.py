import jderobot

class NavdataI(jderobot.Navdata):

    def __init__(self):
        self.data = jderobot.NavdataData()

    def getNavdata(self, current=None):
        return self.data

    def setNavdata(self, data):
        self.data=data