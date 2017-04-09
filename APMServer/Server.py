from __future__ import print_function
import threading, time, sys, traceback
import jderobot, Ice
from pymavlink import mavutil, quaternion, mavwp
from pymavlink.dialects.v10 import ardupilotmega as mavlink
from interfaces.Pose3DI import Pose3DI
from interfaces.NavdataI import NavdataI
from interfaces.MissionI import MissionI
from interfaces.Extra import ExtraI
from pymavlink.quaternion import Quaternion, QuaternionBase

__author__ = 'Jose Antonio Fernandez Casillas'
# Version 1.0



RATE = 50

class Server:


    def __init__(self, port, baudrate):
        """
        In the constructor we create the connection to the APM device
        and start 2 thread one for the comuniction with the device and the other
        one to serve it on Ice
        @:type port: text
        @:param port: port to establish the connection
        @:type baudrate: text
        @:param baudrate: onnection speed
        """

        # control variables to identify if is real the measure
        self.attitudeStatus = 0
        self.altitudeStatus = 0
        self.gpsStatus = 0
        self.lastSentHeartbeat = 0
        self.battery_remainingStatus = 0
        self.rawIMUStatus = 0
        self.scaled_presureStatus = 0

        self.pose3D = Pose3DI(0, 0, 0, 0, 0, 0, 0, 0)
        self.poseWP = Pose3DI(0, 0, 0, 0, 0, 0, 0, 0)
        self.mission = MissionI()
        self.lastMission = MissionI()
        self.oldPoseWP = Pose3DI(0, 0, 0, 0, 0, 0, 0, 0)
        self.navdata = NavdataI()
        self.extra = ExtraI()

        # connect to tu the APM
        self.master = mavutil.mavlink_connection(port, baudrate, autoreconnect=True)
        print('Connection established to device')

        self.master.wait_heartbeat()
        print("Heartbeat Recieved")

        # Set the complete set of commands
        self.master.mav.request_data_stream_send(self.master.target_system,
                                                 self.master.target_component,
                                                 mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                 RATE, 1)

        # Thread to mannage the AMP messages
        MsgHandler = threading.Thread(target=self.mavMsgHandler, args=(self.master,), name='msg_Handler')
        print('Initiating server...')
        # MsgHandler.daemon = True
        MsgHandler.start()


        # Thread to serve Pose3D with the attitude
        print("- Pose3D Ice Server...")
        PoseTheading = threading.Thread(target=self.openPose3DChannel, args=(self.pose3D,), name='Pose_Theading')
        PoseTheading.daemon = True
        PoseTheading.start()
        print("Pose3D Ice Server Up")

        # DEPRECATED
        # Thread to recieve Pose3D with a waypoint
        # PoseTheadingwP = threading.Thread(target=self.openPose3DChannelWP, name='WayPoint_client')
        # PoseTheadingwP.daemon = True
        # PoseTheadingwP.start()


        # Thread to serve Navdata with the all navigation info
        print("- navData Ice Server...")
        NavDataTheading = threading.Thread(target=self.openNavdataChannel, args=(self.navdata,), name='Navdata_Theading')
        NavDataTheading.daemon = True
        NavDataTheading.start()
        print("Navdata Ice Server Up")

        print("- Mission Ice Server...")
        # Thread to recieve a missin plano
        MissionTheading = threading.Thread(target=self.openMissionChannel, args=(self.mission,), name='Mission_Theading')
        MissionTheading.daemon = True
        MissionTheading.start()
        print("Mission Ice Server Up")

        # WP listener DEPRECATED
        # WPListener = threading.Thread(target=self.wpListener, name='WPListener')
        # WPListener.daemon = True
        # WPListener.start()


        print("- Mission listener")
        MissionListener = threading.Thread(target=self.missionListener, name='MissionListener')
        MissionListener.daemon = True
        MissionListener.start()
        print("Mission listener up")

        #  Open the Extra channel in a thread
        print("- Extra Ice Server...")
        ExtraTheading = threading.Thread(target=self.openExtraChannel, args=(self.extra,), name='Extra_Theading')
        ExtraTheading.daemon = True
        ExtraTheading.start()
        print("Extra Ice Server Up")


    def mavMsgHandler(self, m):
        """
        Funtion who handle the mavLink's messages received and refresh the attitude
        :param m: mavLink Connector
        :return: none
        """
        while True:
            msg = m.recv_msg()
            # send heartbeats to autopilot
            if time.time() - self.lastSentHeartbeat > 1.0:
                self.master.mav.heartbeat_send(mavlink.MAV_TYPE_GCS, mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                self.lastSentHeartbeat = time.time()

            if msg is None or msg.get_type() == "BAD_DATA":
                time.sleep(0.01)
                continue

            #print(self.mission.getMissionData())
            #refresh the attitude
            self.refreshAPMPose3D()
            self.refreshAPMnavdata()



    def refreshAPMPose3D(self):
        """
        Funtion to refresh the Pose3D class atribute
        :return: none
        """

        # get attitude of APM
        if 'ATTITUDE' not in self.master.messages:
            self.attitudeStatus = 1
            q=[0,0,0,0]
        else:
            attitude = self.master.messages['ATTITUDE']
            # print(attitude)
            yaw = getattr(attitude,"yaw")
            pitch = getattr(attitude,"pitch") * -1
            roll = getattr(attitude,"roll")
            q = quaternion.Quaternion([roll, pitch, yaw])

        # get altitude of APM
        altitude = self.master.field('VFR_HUD', 'alt', None)
        if altitude is None:
            self.altitudeStatus = 1

        # get GPS position from APM
        latitude = 0
        longitude = 0
        if 'GPS_RAW_INT' not in self.master.messages:
            self.gpsStatus = 1
        else:
            gps = self.master.messages['GPS_RAW_INT']

            latitude = getattr(gps,"lat")/ 10e6
            longitude = getattr(gps,"lon") / 10e6
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

    def refreshAPMnavdata(self):
        """
        Function that apdate
        :return: none
        """
        battery_remaining = 0
        rawIMU = {}
        scaled_presure = {}
        wind = {}
        global_position = {}


        #get battery_remaining
        if 'SYS_STATUS' not in self.master.messages:
            self.battery_remainingStatus = 1
        else:
            stats = self.master.messages['SYS_STATUS']
            battery_remaining = getattr(stats,"battery_remaining")

        #get RAW_IMU
        if 'RAW_IMU' not in self.master.messages:
            self.rawIMUStatus = 1
        else:
            rawIMU = self.master.messages['RAW_IMU']

        #get SCALED PRESSURE
        if 'SCALED_PRESSURE' not in self.master.messages:
            self.scaled_presureStatus = 1
        else:
            scaled_presure = self.master.messages['SCALED_PRESSURE']

        #get WIND
        if 'WIND' not in self.master.messages:
            self.gpsStatus = 1
        else:
            wind = self.master.messages['WIND']

        #get GLOBAL_POSITION_INT
        if 'GLOBAL_POSITION_INT' not in self.master.messages:
            self.gpsStatus = 1
        else:
            global_position = self.master.messages['GLOBAL_POSITION_INT']

        # refresh the navdata
        ndata = jderobot.NavdataData()

        ndata.batteryPercent = battery_remaining
        #print(str(ndata.batteryPercent))
        try:
            ndata.pressure = getattr(scaled_presure, "press_abs")
        except:
            print (str(scaled_presure))
        try:
            ndata.temp = getattr(scaled_presure, "temperature")/100
        except:
            print(str(scaled_presure))
        try:
            ndata.windSpeed = getattr(wind, "speed")
        except:
            print(str(wind))
        try:
            ndata.windAngle = getattr(wind, "direction")
        except:
            print(str(wind))
        try:
            ndata.vx = getattr(global_position, "vx")
        except:
            print(str(global_position))
        try:
            ndata.vy = getattr(global_position, "vy")
        except:
            print(str(global_position))
        try:
            ndata.vz = getattr(global_position, "vz")
        except:
            print(str(global_position))
        try:
            ndata.rotx = getattr(rawIMU, "xgyro")
        except:
            print(str(rawIMU))
        try:
            ndata.roty = getattr(rawIMU, "ygyro")
        except:
            print(str(rawIMU))
        try:
            ndata.rotz = getattr(rawIMU, "zgyro")
        except:
            print(str(rawIMU))
        try:
            ndata.ax = getattr(rawIMU, "xacc")
        except:
            print(str(rawIMU))
        try:
            ndata.ay = getattr(rawIMU, "yacc")
        except:
            print(str(rawIMU))
        try:
            ndata.az = getattr(rawIMU, "zacc")
        except:
            print(str(rawIMU))
        try:
            ndata.magx = getattr(rawIMU, "xmag")
        except:
            print(str(rawIMU))
        try:
            ndata.magy = getattr(rawIMU, "ymag")
        except:
            print(str(rawIMU))
        try:
            ndata.magz = getattr(rawIMU, "zmag")
        except:
            print(str(rawIMU))

        ndata.tagsCount = 0
        ndata.tagsType
        ndata.tagsXc
        ndata.tagsYc
        ndata.tagsWidth
        ndata.tagsHeight
        ndata.tagsOrientation
        ndata.tagsDistance
        ndata.vehicle = 1
        ndata.state = 1

        self.navdata.setNavdata(ndata)


    def oneWaypointMission(self, pose3D):
        '''
        Create a list os one pose3D. This funtion was created to support mission when only a pose3D is recieved.
        At the moment Jderobot does not support a list of Pose3D served in Ice we need another interface
        :param pose3D: a waypoint recieved in Pose3D interface
        :return: none
        '''
        listM = []
        listM.append(pose3D)
        self.setMission(listM)

    def setMission(self, mission):
        '''
        SetUp a mission with a list of waypoints, based on Colorado University Boulder Code
        http://www.colorado.edu/recuv/2015/05/25/mavlink-protocol-waypoints
        :param pose3Dwaypoints: list of waypoints to the mission
        :return: None
        '''
        wp = mavwp.MAVWPLoader()
        seq = 1
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        radius = 10
        pose3Dwaypoints = mission.mission
        print("waypoints received {0}" ,len(pose3Dwaypoints))
        print(self.extra.takeOffDecision)
        N = len(pose3Dwaypoints)
        # Look if a Take Off message has been recieved to set up in the mission too, Take off must to be
        # the fist so if we have the TOFF message we have to create the message and start the loop in 1
        if (self.extra.takeOffDecision):
            navData = pose3Dwaypoints[seq-1]
            toff = mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system,
                                                                self.master.target_component,
                                                                seq,
                                                                frame,
                                                                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # 22
                                                                0, 0, 0, radius, 0, 0,
                                                                navData.x, navData.y, navData.h)
            wp.add(toff)
            # Workaround
            wp.add(toff)
            print(toff)
            seq += 1
            self.extra.setTakeOff(False)
        self.extra.setTakeOff(False)

        for i in range(N):
            navData = pose3Dwaypoints[i]
            wayPoint_tmp =mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system,
                                                                self.master.target_component,
                                                                seq,
                                                                frame,
                                                                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # 16
                                                                0, 0, 0, radius, 0, 0,
                                                                navData.x, navData.y, navData.h)
            wp.add(wayPoint_tmp)
            seq += 1
            print(wayPoint_tmp)

        # Look if a land message has been recieved to set up in the mission too, Land must to be
        # the last so if we have the land message we have to create the message and append to the mission
        print("Land ", self.extra.landDecision)
        if (self.extra.landDecision):
            navData = pose3Dwaypoints[N-1]
            land = mavutil.mavlink.MAVLink_mission_item_message(self.master.target_system,
                                                                self.master.target_component,
                                                                seq,
                                                                frame,
                                                                mavutil.mavlink.MAV_CMD_NAV_LAND, # 21
                                                                0, 0, 0, radius, 0, 0,
                                                                navData.x, navData.y, navData.h)
            seq += 1
            i += 1
            wp.add(land)
            print(land)


        self.master.waypoint_clear_all_send()
        self.master.waypoint_count_send(wp.count())

        for i in range(wp.count()):
            msg = self.master.recv_match(type=['MISSION_REQUEST'], blocking=True)
            print(msg)
            self.master.mav.send(wp.wp(i))
            print ('Sending waypoint {0}'.format(i) + format(wp.wp(msg.seq)))

        self.master.arducopter_arm()
        self.master.set_mode_auto() # arms and start mission I thought

        print('SENDED')
        empty_mission = jderobot.MissionData()
        self.mission.setMissionData(empty_mission)


    # ------------ Ice clients servers ---------------

    def openPose3DChannel(self, pose3D):
        """
        Open a Ice Server to serve Pose3D objects
        :return: none
        """

        status = 0
        ic = None
        # recovering the attitude
        Pose2Tx = pose3D
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


    def openPose3DChannelWP(self):
        '''
        Open a Pose3D client to recieve Pose3D with a waypoint
        :return:  mone
        '''
        status = 0
        ic = None
        pose2Rx = self.poseWP
        try:
            ic = Ice.initialize(sys.argv)
            adapter = ic.createObjectAdapterWithEndpoints("WPAdapter", "default -p 9992")
            object = pose2Rx
            # print object.getPose3DData()
            adapter.add(object, ic.stringToIdentity("WP_pose3d"))
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


    def openMissionChannel(self, mission):
        '''
        Open a Mission client to recieve Mission with a waypoint
        :return:  mone
        '''
        print("mission Ice Server")
        status = 0
        ic = None
        pose2Rx = mission

        try:
            ic = Ice.initialize(sys.argv)
            adapter = ic.createObjectAdapterWithEndpoints("MissionAdapter", "default -p 10008")
            object = pose2Rx
            adapter.add(object, ic.stringToIdentity("ardrone_mission"))
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


    def openNavdataChannel(self, navdata):
        '''
        Open a Ice Server to serve all the navigation data
        :return:
        '''
        status = 0
        ic = None
        navdata2Tx = navdata

        try:
            ic = Ice.initialize(sys.argv)
            adapter = ic.createObjectAdapterWithEndpoints("NavdataAdapter", "default -p 9996")
            object = navdata2Tx
            adapter.add(object, ic.stringToIdentity("ardrone_navdata"))
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

    def openExtraChannel(self, extra):

        status = 0
        ic = None
        extra_tx = extra
        try:
            ic = Ice.initialize(sys.argv)
            adapter = ic.createObjectAdapterWithEndpoints("extra_adapter", "default -p 9994")
            object = extra_tx
            adapter.add(object, ic.stringToIdentity("ardrone_extra"))
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


    def wpListener(self):
        '''
        DEPRECATED listen to a WP recieved in Ice to go to
        :return: None
        '''
        while True:
            if not self.poseWP.equals(self.oldPoseWP):
                self.oldPoseWP.setPose3DData(self.poseWP.getPose3DData())
                self.oneWaypointMission(self.poseWP)
                time.sleep(1)

    def missionListener(self):
        '''
        Function who listen to a new mission revieved from Ice MissionChannel thread and send it to APM
        :return: None
        '''
        while True:
            if not self.mission.is_empty():
                self.lastMission = self.mission
                self.setMission(self.mission.getMissionData())

            time.sleep(1)
