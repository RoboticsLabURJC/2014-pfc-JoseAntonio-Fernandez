__author__ = 'Jose Antonio Fernandez Casillas'

import Server, threading
import Ice, sys
import traceback
from interfaces import Pose3DI, CMDVelI

'''boot secuence and  open the serverData adquisiton channel'''

def openPose3DChannel(Pose3D):

    status = 0
    ic = None
    #recojo el pose3D del servidor
    Pose2Tx = Server.getPose3D
    try:
        ic = Ice.initialize(sys.argv)
        adapter = ic.createObjectAdapterWithEndpoints("Pose3DAdapter", "default -p 9998")
        object = Pose2Tx
        #print object.getPose3DData()
        adapter.add(object, ic.stringToIdentity("Pose3D"))
        adapter.activate()
        ic.waitForShutdown()
    except:
        traceback.print_exc()
        status = 1

#TODO abrir hilo con el Server

PoseTheading = threading.Thread(target=Server, args=("/dev/ttyUSB0", 57600), name='Pose_Theading')
PoseTheading.daemon = True
PoseTheading.start()

#PROBAR TODO abrir hilo que sirva el Pose3D en Ice
argPose3D = Pose3DI(0, 0, 0, 0, 0, 0, 0, 0)  # 1 to avoid indeterminations AeroCano Code
PoseTheading = threading.Thread(target=openPose3DChannel, args=(argPose3D), name='Pose_Theading') #no puede llamarse igual los 2 hilos
PoseTheading.daemon = True
PoseTheading.start()


    # TODO abrir hilo de actuación esperando ordenes