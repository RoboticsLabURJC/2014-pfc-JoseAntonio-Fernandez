from MapClient.classes import Pose3DI

class WayPoint:
    def __init__(self, x=0, y=0, lat=0, lon=0, h=0):
        self.x = x
        self.y = y
        self.h = h
        self.lat= lat
        self.lon = lon
        #todo mejorar para que se calculen automagicamente

    @staticmethod
    def waypoint_to_pose(self, awaypoint):
        pose = Pose3DI(0,0,0,0,0,0,0,0)
        data = pose.getPose3DData()
        data.x = awaypoint.lon
        data.y = awaypoint.lat
        data.h = awaypoint.h
        pose.setPose3DData(data)
        return pose
