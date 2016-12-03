"""
DEPRECATED
"""
import ardupilot,time
import sys, traceback, Ice, random
import jderobot
from pymavlink import quaternion

class Pose3DI(jderobot.Pose3D):
    def getPose3DData(self, current=None):

        while not ac.isArmed():
            time.sleep(0.5)

        pose=jderobot.Pose3DData()
        att = ac.getAttitude()
        gps = ac.getLocation()
        print ("attitude" + str(att))
        print ("gps" + str(gps))
        yaw = att["yaw"]
        pitch = att["pitch"]
        roll = att["roll"]
        pose.x = gps["lat"]
        pose.y = gps["lon"]
        pose.z = ac.getAltitude()

        q = quaternion.Quaternion([roll,pitch,yaw])
        pose.q0 = q.__getitem__(0)
        pose.q1 = q.__getitem__(1)
        pose.q2 = q.__getitem__(2)
        pose.q3 = q.__getitem__(3)

        print (q)

        return pose

    def setPose3DData(self,pose):
        print (pose)


port = "/dev/ttyUSB0"
ac = ardupilot.ArduPilot(port, 57600)
ac.arm()
print (ac.isArmed())


status = 0
ic = None
try:
    ic = Ice.initialize(sys.argv)
    adapter = ic.createObjectAdapterWithEndpoints("ImuPluginAdapter", "default -p 9998")
    object = Pose3DI()
    adapter.add(object, ic.stringToIdentity("ImuPlugin"))
    adapter.activate()
    ic.waitForShutdown()
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

