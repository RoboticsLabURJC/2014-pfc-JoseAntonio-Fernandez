from __future__ import print_function
__author__ = 'Jose Antonio Fernandez Casillas'



import threading
import time
import jderobot
from pymavlink import mavutil, quaternion
from pymavlink.dialects.v10 import ardupilotmega as mavlink


class Server:
    def __init__(self, port, baudrate):
        '''control variables to identify if is real the measure'''
        self.attitudeStatus = 0
        self.altitudeStatus = 0
        self.gpsStatus = 0
        self.lastSentHeartbeat = 0
        self.pose3D = jderobot.Pose3DData()


        '''Conectar al Ardupilot'''
        self.master = mavutil.mavlink_connection(port, baudrate, autoreconnect=True)
        print('Connection established to device...')

        self.master.wait_heartbeat()
        print("Heartbeat Recieved")

        T = threading.Thread(target=self.mavMsgHandler, args=(self.master,))
        print('Initiating server')
        T.start()

    def mavMsgHandler(self, m):
        while True:
            msg = m.recv_msg()
            # print msg
            # send heartbeats to autopilot
            if time.time() - self.lastSentHeartbeat > 1.0:
                self.master.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.lastSentHeartbeat = time.time()

            if msg is None or msg.get_type() == "BAD_DATA":
                time.sleep(0.01)
                continue

            # enable data streams after start up - can't see another way of doing this.
            if msg.get_type() == "STATUSTEXT" and "START" in msg.text:
                self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTRA1)
                self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTENDED_STATUS)
                self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTRA2)
                self.setDataStreams(mavlink.MAV_DATA_STREAM_POSITION)

            #if "ACK" in msg.get_type():
            #    print(msg)

            '''Compruebo la actitud'''
            self.refreshAPMPose3D()

    def refreshAPMPose3D(self):
        '''get attitude of APM'''
        if 'ATTITUDE' not in self.master.messages:
            self.attitudeStatus = 1
        else:
            attitude = self.master.messages['ATTITUDE']
            print(attitude)
            yaw = getattr(attitude,"yaw")
            pitch = getattr(attitude,"pitch")
            roll = getattr(attitude,"roll")
            q = quaternion.Quaternion([roll, pitch, yaw])

        '''get altitude of APM'''
        altitude = self.master.field('VFR_HUD', 'alt', None)
        if altitude is None:
            self.altitudeStatus = 1

        '''get GPS position from APM'''
        latitude = 0
        longitude = 0
        if 'GPS_RAW_INT' not in self.master.messages:
            self.gpsStatus = 1
        else:
            gps = self.master.messages['GPS_RAW_INT']
            # TODO por que dividir entre 10e6

            latitude = getattr(gps,"lat")/ 10e6;
            longitude = getattr(gps,"lon") / 10e6;
            self.GPS_fix_type = getattr(gps,"fix_type")

        # refresh the pose3D
        self.pose3D.x = latitude
        self.pose3D.y = longitude
        self.pose3D.z = altitude
        self.pose3D.q0 = q.__getitem__(0)
        self.pose3D.q1 = q.__getitem__(1)
        self.pose3D.q2 = q.__getitem__(2)
        self.pose3D.q3 = q.__getitem__(3)

    def getPose3D(self):
        return self.pose3D

    #TODO revisar actuacion hay que poner puntos y luego empezar o se debe ir uno a uno
    #TODO revisar actuacion hay que poner puntos y luego empezar o se debe ir uno a uno
    # funcion GotoXY(Pose3D)

    def flyTo(self, lat, lon, alt): #TODO revisar con el codigo de Jorge Cano/Vela
        #																				seqfrm cmd                          cur at p1 p2 p3 p4  x    y    z
        self.master.mav.mission_item_send(self.mav.target_system, self.mav.target_component, 0, 0, mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 5, 0, 0, 0, lat, lon, alt)

test = Server("/dev/ttyUSB0", 57600)