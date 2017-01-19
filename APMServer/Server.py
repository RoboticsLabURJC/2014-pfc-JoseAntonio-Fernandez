from __future__ import print_function
__author__ = 'Jose Antonio Fernandez Casillas'
#Version 1.0


import threading,time,sys,traceback
import jderobot,Ice
from pymavlink import mavutil, quaternion, mavwp
from pymavlink.dialects.v10 import ardupilotmega as mavlink
from interfaces.Pose3DI import  Pose3DI



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

        MsgHandler = threading.Thread(target=self.mavMsgHandler, args=(self.master,), name='msg_Handler')
        print('Initiating server...')
        #MsgHandler.daemon = True
        MsgHandler.start()


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
            msg = str(m.recv_msg())
            # print msg
            # send heartbeats to autopilot
            if time.time() - self.lastSentHeartbeat > 1.0:
                self.master.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.lastSentHeartbeat = time.time()

            if msg is None or msg.get_type() == "BAD_DATA":
                time.sleep(0.01)
                continue
            '''
            # enable data streams after start up - can't see another way of doing this.
            if msg.get_type() == "STATUSTEXT" and "START" in msg.text:
                self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTRA1)
                self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTENDED_STATUS)
                self.setDataStreams(mavlink.MAV_DATA_STREAM_EXTRA2)
                self.setDataStreams(mavlink.MAV_DATA_STREAM_POSITION)
                #self._master.mav.request_data_stream_send(0, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL,4, 1)
            '''
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
        """
        Open a Ice Server to serv Pose3D objects
        :return: none
        """

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


    def flyTo(self, lat, lon, alt):
        #TODO revisar con el codigo de Jorge Cano/Vela
        #TODO cambiar signatura por navigateTo(self, pose3D)
        #																				seqfrm cmd                          cur at p1 p2 p3 p4  x    y    z
        self.master.mav.mission_item_send(self.mav.target_system, self.mav.target_component, 0, 0, mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 5, 0, 0, 0, lat, lon, alt)

    def setMission(self, pose3Dwaypoints):
        '''
        SetUp a mission with a list of waypoints, based on Colorado University Boulder Code
        http://www.colorado.edu/recuv/2015/05/25/mavlink-protocol-waypoints
        :param pose3Dwaypoints: list of waypoints to the mission
        :return: None
        '''
        wp = mavwp.MAVWPLoader()
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        radius = 20
        N = pose3Dwaypoints.length
        for i in range(N):
            navData = jderobot.Pose3DData()
            navData.setPose3DData(pose3Dwaypoints[i].getPose3DData)
            wp.add(mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system,
                                                                self.master.target_component,
                                                                i,
                                                                frame,
                                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                                0, 0, 0, radius, 0, 0,
                                                                navData.x, navData.y, navData.h))
        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp.count())


        for i in range(wp.count()):
            msg = self.master.recv_match(type=['MISSION_REQUEST'], blocking=True)
            self.master.mav.send(wp.wp(msg.seq))
            print ('Sending waypoint {0}'.format(msg.seq))

        self.master.set_mode_auto() # arms and start mission I thought
        