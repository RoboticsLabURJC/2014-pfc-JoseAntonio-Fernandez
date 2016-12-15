from __future__ import print_function
__author__ = 'Jose Antonio Fernandez Casillas'



import threading,time,sys,traceback
import jderobot,Ice
from pymavlink import mavutil, quaternion
from pymavlink.dialects.v10 import ardupilotmega as mavlink
from APMServer.interfaces.Pose3DI import  Pose3DI



class Server:

    def __init__(self, port, baudrate):
        """
        In the constructor we create the connection to the APM device
        and start 2 thread one for the comuniction with the device and the other
        one to serv it on Ice
        @:type port: text
        @:param port: port to establish the connection
        @:type baudrate: text
        @:param baudrate: onnection speed
        """

        #control variables to identify if is real the measure
        self.attitudeStatus = 0
        self.altitudeStatus = 0
        self.gpsStatus = 0
        self.lastSentHeartbeat = 0
        self.pose3D = Pose3DI(0,0,0,0,0,0,0,0)
        #TODO cambiar el Pose3DData por un Pose 3D

        '''Conectar al Ardupilot'''
        self.master = mavutil.mavlink_connection(port, baudrate, autoreconnect=True)
        print('Connection established to device')

        self.master.wait_heartbeat()
        print("Heartbeat Recieved")

        T = threading.Thread(target=self.mavMsgHandler, args=(self.master,))
        print('Initiating server...')
        T.start()


        PoseTheading = threading.Thread(target=self.openPose3DChannel, args=(self.pose3D,), name='Pose_Theading')
        PoseTheading.daemon = True
        PoseTheading.start()


    def mavMsgHandler(self, m):
        """
        Funtion who handle the mavLink's messages received and refresh the attitude
        :param m: mavLink Connector
        :return: none
        """
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


            #refresh the attitude
            self.refreshAPMPose3D()

    def refreshAPMPose3D(self):
        """
        Funtion to refresh the Pose3D class atribute
        The altitude is recovered to but already is not used
        :return: none
        """

        #get attitude of APM
        if 'ATTITUDE' not in self.master.messages:
            self.attitudeStatus = 1
            q=[0,0,0,0]
        else:
            attitude = self.master.messages['ATTITUDE']
            #print(attitude)
            yaw = getattr(attitude,"yaw")
            pitch = getattr(attitude,"pitch")
            roll = getattr(attitude,"roll")
            q = quaternion.Quaternion([roll, pitch, yaw])

        #get altitude of APM
        altitude = self.master.field('VFR_HUD', 'alt', None)
        if altitude is None:
            self.altitudeStatus = 1

        #get GPS position from APM
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
        data = jderobot.Pose3DData
        data.x = latitude
        data.y = longitude
        data.z = altitude
        data.h = altitude
        data.q0 = q.__getitem__(0)
        data.q1 = q.__getitem__(1)
        data.q2 = q.__getitem__(2)
        data.q3 = q.__getitem__(3)
        self.pose3D.setPose3DData(data)

    def openPose3DChannel(self, pose3D):
        '''
        Open a Ice Server to serv Pose3D objects
        :param pose3D: the pose to serv
        :return: None
        '''

        status = 0
        ic = None
        # recovering the attitude
        Pose2Tx = pose3D
        print('Open the Ice Server Channel')
        try:
            ic = Ice.initialize(sys.argv)
            adapter = ic.createObjectAdapterWithEndpoints("Pose3DAdapter", "default -p 9998")
            object = Pose2Tx
            # print object.getPose3DData()
            adapter.add(object, ic.stringToIdentity("ardrone_pose3d")) #ardrone_pose3d  Pose3D
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

    #TODO revisar actuacion hay que poner puntos y luego empezar o se debe ir uno a uno
    # funcion GotoXY(Pose3D)

    def flyTo(self, lat, lon, alt): #TODO revisar con el codigo de Jorge Cano/Vela
        #																				seqfrm cmd                          cur at p1 p2 p3 p4  x    y    z
        self.master.mav.mission_item_send(self.mav.target_system, self.mav.target_component, 0, 0, mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 5, 0, 0, 0, lat, lon, alt)
