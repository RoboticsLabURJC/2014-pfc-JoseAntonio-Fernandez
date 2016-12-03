__author__ = 'Jose Antonio Fernandez Casillas'

import threading
from APMServer import Server





'''boot secuence and  open the serverData adquisiton channel'''



#TODO abrir hilo con el Server
print('abro hilo Servidor')
serverTh = threading.Thread(target=Server, args=("/dev/ttyUSB0", 57600), name='Server')
serverTh.daemon = True
serverTh.start()

'''
print('abro hilo Ice')
#PROBAR TODO abrir hilo que sirva el Pose3D en Ice
#argPose3D = Pose3DI.Pose3DI(0,0,0,0,0,0,0,0)  # 1 to avoid indeterminations AeroCano Code
PoseTheading = threading.Thread(target=openPose3DChannel(), name='Pose_Theading')
PoseTheading.daemon = True
PoseTheading.start()
'''

test = Server("/dev/ttyUSB0", 57600)