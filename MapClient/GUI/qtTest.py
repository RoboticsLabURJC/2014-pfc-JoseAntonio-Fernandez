import jderobot

from MapClient.GUI.sensorsWidget import SensorsWidget
from MapClient.tools import GeoUtils
from PyQt5 import QtCore
from PyQt5.QtCore import QSize
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import *
import threading
import cv2
import numpy as np
import math

from MapClient.GUI.cameraWidget import CameraWidget
from MapClient.tools import ImageUtils, WayPoint

from MapClient.ice import ice_init

LEFT =1
RADIUS = 0.5
WIDTH = 600.0
HEIGHT = 600.0
ZOOM = 16


class MainGUI(QWidget):
    updGUI = pyqtSignal()
    update_map = pyqtSignal()


    def __init__(self, imageInput, parent=None):
        super(MainGUI, self).__init__(parent)

        self.setFixedHeight(680)

        self.pose = None
        self.updGUI.connect(self.updateGUI)
        self.update_map.connect(self.update_position)

        #Set the image Metadata
        self.imageMetadata = imageInput
        self.wayPoints = []
        self.wayPoints_lat_lon = []
        self.limit_warning = False
        self.initial_pose = jderobot.Pose3DData

        #Prepare the image to be showed
        image = self.imageMetadata["bytes"]
        cvRGBImg = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        #self.image2show = ImageUtils.prepareInitialImage(self.imageMetadata["bytes"], self.imageMetadata["size"][0],self.imageMetadata["size"][1])
        self.cvImage = cvRGBImg.copy()
        self.cvImageShadow = cvRGBImg.copy()
        self.im_to_show = cvRGBImg.copy()

        height, width, channel = self.cvImage.shape
        bytesPerLine = 3 * width


        qimg = QImage(cvRGBImg, width, height, bytesPerLine, QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)


        self.imageLabel = QLabel()
        self.imageLabel.setFixedSize(600,600)
        self.imageLabel.setPixmap(qpm)
        self.imageLabel.setMouseTracking(True)
        self.imageLabel.mouseMoveEvent = self.getPos
        self.imageLabel.mousePressEvent= self.addWayPoint

        self.mapLayout = QGridLayout()
        self.newMap = QPushButton("Retrieve new map")
        self.newMap.clicked.connect(lambda: self.newMapPush(self.newMap))

        self.mapSourceGoogle = QRadioButton("Google")
        self.mapSourceIGN = QRadioButton("IGN")
        self.mapSourceIGN.setChecked(True)
        self.mapLayout.addWidget(self.mapSourceIGN,0,0)
        self.mapLayout.addWidget(self.mapSourceGoogle,0,1)

        self.mapLayout.addWidget(self.newMap,0,2)

        #Prepared to manual control of copters
        self.tabs = QTabWidget()
        self.missionTab = QWidget()
        self.manualTab= QWidget()
        self.tabs.addTab(self.missionTab,"Mission")
        self.tabs.addTab(self.manualTab,"Manual")

        self.labelGP = QLabel()
        self.table = QTableWidget()
        self.table.setColumnCount(1)
        self.table.setRowCount(0)
        self.table.insertColumn(0)

        itemCol = QTableWidgetItem("Waypoint")
        altitude = QTableWidgetItem("Altitude")
        self.table.setHorizontalHeaderItem(0,itemCol)
        self.table.setHorizontalHeaderItem(1,altitude)
        self.table.setMinimumSize(360,100)
        self.table.setColumnWidth(0, 340 * 0.75) # 25% Width Column
        self.table.setColumnWidth(1, 340 * 0.25) # 75% Width Column
        header = self.table.horizontalHeader()
        header.setStretchLastSection(True)

        self.buttonLayout = QVBoxLayout()
        self.altitudeText = QLineEdit("40")

        self.frameType = QGroupBox("UAV type:")
        self.vbox = QVBoxLayout()
        self.planeFrame = QRadioButton("Plane")
        self.copterFrame =QRadioButton("Copter")
        self.vbox.addWidget(self.planeFrame)
        self.vbox.addWidget(self.copterFrame)
        self.frameType.setLayout(self.vbox)

        self.toffButton = QPushButton("Take off")
        self.toffButton.setCheckable(True)
        self.toffButton.clicked.connect(lambda: self.setTakeOffLand(self.toffButton))
        self.toffButton.setChecked(False)
        self.toffButton.setFixedSize(140,100)
        self.send2APM = QPushButton("Send")
        self.send2APM.clicked.connect(lambda: self.sendWP(self.send2APM))
        self.clear_mission_button = QPushButton("Clear Mission")
        self.clear_mission_button.clicked.connect(lambda: self.clear_mission())
        self.add_waypoint_button = QPushButton("Add waypoint")
        self.add_waypoint_button.clicked.connect(lambda: self.add_waypoint_table())
        self.send2APM.setFixedSize(140,100)
        self.cameraCheck = QCheckBox("Camera")
        self.cameraCheck.stateChanged.connect(self.showCameraWidget)
        self.attitudeCheck = QCheckBox("Attitude")
        self.attitudeCheck.stateChanged.connect(self.showSensorsWidget)


        self.buttonLayout.addSpacing(2)
        self.labelAltitude = QLabel("Default altitude:")
        self.buttonLayout.addWidget(self.labelAltitude)
        self.buttonLayout.addWidget(self.altitudeText)
        self.buttonLayout.addWidget(self.frameType)
        self.buttonLayout.addWidget(self.add_waypoint_button)
        self.buttonLayout.addWidget(self.clear_mission_button)
        self.planeFrame.setChecked(True)


        self.buttonLayout.addSpacing(160)
        self.buttonLayout.addWidget(self.toffButton)
        self.buttonLayout.addWidget(self.send2APM)
        self.buttonLayout.addWidget(self.cameraCheck)
        self.buttonLayout.addWidget(self.attitudeCheck)

        layoutPpal = QGridLayout()
        layoutPpal.addWidget(self.imageLabel)
        layoutPpal.addLayout(self.mapLayout,1,0)
        layoutPpal.addWidget(self.table,0,1)
        layoutPpal.addLayout(self.buttonLayout, 0, 2)
        #layoutPpal.addWidget(self.labelXY)
        layoutPpal.addWidget(self.labelGP)
        layoutPpal.addWidget(self.labelGP)



        mainLayout = QGridLayout()
        mainLayout.addLayout(layoutPpal, 0, 1)

        self.setLayout(mainLayout)
        self.setWindowTitle("AUVCommander")

        # Widgets
        self.cameraWidget = CameraWidget(self)
        self.sensorsWidget = SensorsWidget(self)

    def clear_mission(self):
        self.cvImageShadow = self.cvImage.copy()
        self.im_to_show = self.im_to_show
        self.refreshImage()
        self.wayPoints = []
        self.wayPoints_lat_lon = []
        self.table.setRowCount(0)
        self.table.clearContents()

    def add_waypoint_table(self):
        dialog = MyDialog(self)
        print('add')
        lat,lon,alt = dialog.exec()
        print(lat)

    def getPos(self, event):
        '''
        function who reference the mouse position in the image over coordinades
        :param event: event executed
        :return: None
        '''
        x = event.pos().x()
        y = event.pos().y()
        lonMin, latMin, lonMax, latMax = self.imageMetadata['bbox']

        sizeX, sizeY = self.imageMetadata['size']

        lat, lon = ImageUtils.posImage2Coords(x, y, sizeX, sizeY, latMin, lonMin, latMax, lonMax)
        self.labelGP.setText("lat: " + str(lat) + ' lon: ' +str(lon) + " X: " + str(x)+ " Y: " + str(y))

    def addWayPoint(self, event):
        x = event.pos().x()
        y = event.pos().y()
        point = (x,y)
        self.wayPoints.append(point)

        if (event.button() == QtCore.Qt.LeftButton):
            lonMin, latMin, lonMax, latMax = self.imageMetadata['bbox']
            sizeX, sizeY = self.imageMetadata['size']
            lat, lon = ImageUtils.posImage2Coords(x, y, sizeX, sizeY, latMin, lonMin, latMax, lonMax)
            point_ll = (lat, lon)
            self.wayPoints_lat_lon.append(point)
            currentRowCount = self.table.rowCount()
            item = QTableWidgetItem()
            alt = QTableWidgetItem()
            if self.toffButton.isChecked():
                if self.toffButton.text() == 'Take off':
                    item.setText("TAKE OFF to " + str(lat) + ' ' +str(lon))
                    self.toffButton.setChecked(False)
                    self.toffButton.setText('Land')
                else:
                    item.setText("LAND in " + str(lat) + ' ' +str(lon))
                    self.toffButton.setChecked(False)
                    self.toffButton.setText('Take off')
            else:
                item.setText("lat: " + str(lat) + ' lon: ' +str(lon))
            alt.setText(self.altitudeText.text())
            self.table.insertRow(currentRowCount)
            self.table.setItem(currentRowCount, 0, item)
            self.table.setItem(currentRowCount, 1, alt)


    def setTakeOffLand(self, button):
        if self.planeFrame.isChecked():
            self.toffButton.setChecked(True)
        else:
            if self.toffButton.text() == 'Take off':
                self.toffButton.setText('Land')
                self.toffButton.setChecked(False)
            else:
                self.toffButton.setText('Take off')
                self.toffButton.setChecked(False)
            #TODO call to takeOff over Ice

    def newMapPush(self, newMap):
        pose = self.getPose3D().getPose3D()

        if (self.mapSourceIGN.isChecked()):
            self.imageMetadata = GeoUtils.retrieve_new_map(pose.x, pose.y, RADIUS, WIDTH, HEIGHT)

            # Prepare the image to be showed
            image = self.imageMetadata["bytes"]
            cvRGBImg = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # self.image2show = ImageUtils.prepareInitialImage(self.imageMetadata["bytes"], self.imageMetadata["size"][0],self.imageMetadata["size"][1])
            self.cvImage = cvRGBImg.copy()
            self.cvImageShadow = cvRGBImg.copy()
            self.im_to_show = cvRGBImg.copy()

            self.refreshImage()

        else:
            self.imageMetadata = GeoUtils.retrieve_new_google_map(pose.x, pose.y, ZOOM, WIDTH, HEIGHT)
            # Prepare the image to be showed
            image = self.imageMetadata["bytes"]
            cvRGBImg = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # self.image2show = ImageUtils.prepareInitialImage(self.imageMetadata["bytes"], self.imageMetadata["size"][0],self.imageMetadata["size"][1])
            self.cvImage = cvRGBImg.copy()
            self.cvImageShadow = cvRGBImg.copy()
            self.im_to_show = cvRGBImg.copy()

            self.refreshImage()

    def sendWP(self, send2APM):
        '''HARDCODED
        item = self.table.item(0,0)
        pose = jderobot.Pose3DData()
        pose.x = -35.362938
        pose.y = 149.165085
        pose.z = 0
        pose.h = 500
        pose.q0 = 0
        pose.q1 = 0
        pose.q2 = 0
        pose.q3 = 0
        '''
        colCount = self.table.rowCount()
        mission = jderobot.MissionData()
        mission.mission = []
        print(mission)
        i = 0
        for row in range(colCount):
            pose = jderobot.Pose3DData(0,0,0,0,0,0,0,0)
            text = self.table.item(row, 0).text()
            pos_lat = text.find("lat:")
            pos_lon = text.find("lon:")
            if pos_lat != -1:
                print("WP")
                lat = (text[pos_lat + 4:pos_lon])
                pose.x = float(lat)
                lon = (text[pos_lon + 4:])
                pose.y = float(lon)
            else:
                if "LAND in " in text:
                    print("LAND")
                    pos = text.find("LAND in ") + 8
                    position = text[pos:].split(" ")
                    lat = position[0]
                    lon = position[1]
                    pose.x = float(lat)
                    pose.y = float(lon)
                    self.extra.land()
                else:
                    pos = text.find("TAKE OFF to ") + 12
                    print("Take of detected")
                    position = text[pos:].split(" ")
                    lat = position[0]
                    lon = position[1]
                    pose.x = float(lat)
                    pose.y = float(lon)
                    self.extra.takeoff()

            alt = self.table.item(row, 1)
            pose.h = int(alt.text())
            print(mission.mission)
            mission.mission.append(pose)
            # Workaround to avoid the fist waypoint un-understanded
            if i==0:
                mission.mission.append(pose)
            # end Workaround
            i += 1

            print(text + alt.text())


        self.mission.setMissionData(mission)
        #send the mission
        print("sending... " + str(self.mission.getMissionData()))

        MissionTheading = threading.Thread(target=ice_init.send_mission, args=(mission_to_send,), name='Mission_Theading')
        MissionTheading.daemon = True
        MissionTheading.start()

        #ice_init.send_mission(mission_to_send)

    def setFirstLocation(self, imageInput):
        '''
        Set the image of the location
        :param imageInput: Georeferenced image and bbox and center GPS position
        :return:
        '''
        self.imageMetadata = imageInput


    def getCamera(self):
        return self.camera

    def setCamera(self, camera):
        self.camera = camera

    def getNavData(self):
        return self.navdata

    def setNavData(self, navdata):
        self.navdata = navdata

    def getPose3D(self):
        return self.pose

    def get_initial_pose3D(self):
        return self.initial_pose

    def set_initial_pose3D(self,pose):
        self.initial_pose = pose

    def setPose3D(self, pose):
        self.pose = pose

    def getCMDVel(self):
        return self.cmdvel

    def setCMDVel(self, cmdvel):
        self.cmdvel = cmdvel

    def getExtra(self):
        return self.extra

    def setExtra(self, extra):
        self.extra = extra

    def getMission(self):
        return self.mission

    def setMission(self, mission):
        self.mission = mission


    def updateGUI(self):
        '''
        Update the position of the UAV in the map and the attitude and camera image in it's own widgets
        :return:
        '''
        # HARDCODED
        self.updateImage()
        self.cameraWidget.imageUpdate.emit()
        self.sensorsWidget.sensorsUpdate.emit()

    # inherit from Alberto Martin Florido UAV Viewer with a few little changes



    def showCameraWidget(self, state):
        if self.cameraCheck.isChecked():
            self.cameraWidget.show()
        else:
            self.cameraWidget.close()

    def closeCameraWidget(self):
        self.cameraCheck.setChecked(False)

    def showSensorsWidget(self, state):
        if self.attitudeCheck.isChecked():
            self.sensorsWidget.show()
        else:
            self.sensorsWidget.close()

    def closeSensorsWidget(self):
        self.attitudeCheck.setChecked(False)


    def closeEvent(self, event):
        self.camera.stop()
#        self.navdata.stop()
        self.pose.stop()
        event.accept()

    # Testing
    def updateImage(self):

        img = self.imageMetadata.get("numpy")
        if img is not None:
            image = QImage(img.data, img.shape[1], img.shape[0], img.shape[1] * img.shape[2], QImage.Format_RGB888);
            if img.shape[1] == self.IMAGE_COLS_MAX:
                x = 20
            else:
                x = (self.IMAGE_COLS_MAX + 20) / 2 - (img.shape[1] / 2)
            if img.shape[0] == self.IMAGE_ROWS_MAX:
                y = 40
            else:
                y = (self.IMAGE_ROWS_MAX + 40) / 2 - (img.shape[0] / 2)
            img_to_show = ImageUtils.refreshPosition(self.image2show, 40.4153774, -3.708283)
            size = QSize(img.shape[1], img.shape[0])
            self.imageLabel.move(x, y)
            self.imageLabel.resize(size)
            self.imageLabel.setPixmap(QPixmap.fromImage(image))

    def refreshImage(self):
        height, width, channel = self.cvImage.shape
        bytesPerLine = 3 * width
        qimg = QImage(self.im_to_show.data, width, height, bytesPerLine, QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.imageLabel.setPixmap(qpm)

    def setPosition(self, x, y,angle):
        self.refresh_shadow(x,y)
        self.set_waypoints(self.wayPoints)
        self.draw_triangle(x, y, angle)
        self.refreshImage()


    def set_waypoints(self, wayPoints, current=True):
        n=0
        pts = np.array(wayPoints, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(self.cvImageShadow, [pts], False, (250, 250, 250), thickness=1)
        for waypoint in wayPoints:
            n+=1
            s = str(n)
            cv2.circle(self.cvImageShadow, (waypoint[0], waypoint[1]), 1, [0, 0, 255], thickness=-1, lineType=1)
            cv2.putText(self.cvImageShadow, s, (waypoint[0] + 3, waypoint[1]), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, [0, 0, 255], thickness=1)
        self.refreshImage()

    def refreshImage(self):
        height, width, channel = self.cvImage.shape
        bytesPerLine = 3 * width
        qimg = QImage(self.im_to_show.data, width, height, bytesPerLine, QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.imageLabel.setPixmap(qpm)

    def change_coordinate_system(self, points, origin, toCartsian=True):
        points_transformed = []
        if toCartsian:
            for corner in points:
                x = corner[0] - origin[0]
                y = origin[1] - corner[1]
                points_transformed.append([x, y])
        else:
            for corner in points:
                x = corner[0] + origin[0]
                y = origin[1] + corner[1]
                points_transformed.append([x, y])

        return points_transformed

    def rotate_polygon(self, polygon, angle):
        """Rotates the given polygon which consists of corners represented as (x,y),
        around the ORIGIN, clock-wise, theta degrees"""
        #theta = math.radians(angle)
        theta = angle
        rotatedPolygon = []
        for corner in polygon:
            rotatedPolygon.append((corner[0] * math.cos(theta) - corner[1] * math.sin(theta),
                                   corner[0] * math.sin(theta) + corner[1] * math.cos(theta)))
        return rotatedPolygon

    def draw_triangle(self, x, y, angle):
        triangle = [[x-5,y-7],[x,y+7],[x+5,y-7]]
        center = self.center_of_triangle(triangle)
        triangleCartesian = self.change_coordinate_system(triangle, center,True)
        rotated_triangleCartesian = self.rotate_polygon(triangleCartesian,angle)
        image_center = [x,y]
        rotated_triangle = self.change_coordinate_system(rotated_triangleCartesian,image_center ,False)
        pts = np.array(rotated_triangle, np.int32)
        pts = pts.reshape((-1,1,2))
        self.im_to_show = self.cvImageShadow.copy()
        cv2.fillPoly(self.im_to_show, [pts], (250,0,0))
        cv2.polylines(self.im_to_show,[pts],True,(250,0,0),thickness=1)

    def refresh_shadow(self,x,y):
        cv2.circle(self.cvImageShadow, (x, y), 1, [255,255,102], thickness=-1, lineType=8, shift=0)

    def center_of_triangle(self, triangle):
        center_x = (triangle[0][0] + triangle[1][0] + triangle[2][0]) / 3;
        center_y = (triangle[0][1]+ triangle[1][1] + triangle[2][1]) / 3;
        return [center_x,center_y]

    def update_position(self):

        if self.pose != None:
            pose = self.getPose3D().getPose3D()
            lat = pose.x
            lon = pose.y
            bbox = self.imageMetadata["bbox"]
            self.limit_warning = GeoUtils.is_position_close_border(lat, lon , bbox)
            if self.limit_warning:
                self.download_zoomed_map()
            else:
                bbox = self.imageMetadata["bbox"]
                imagesize = self.imageMetadata["size"]
                x, y = ImageUtils.posCoords2Image(bbox[0], bbox[1], bbox[2], bbox[3], lat, lon, imagesize[0], imagesize[1])
                angle = self.sensorsWidget.quatToYaw(pose.q0, pose.q1, pose.q2, pose.q3)
                self.setPosition(x, y, angle)

    def download_zoomed_map(self):
        poseI = self.get_initial_pose3D()
        latI = poseI.x
        lonI = poseI.y
        if self.mapSourceIGN.isChecked():
            self.imageMetadata = GeoUtils.retrieve_new_map(latI, lonI, RADIUS * 2, WIDTH, HEIGHT)
        else:
            self.imageMetadata = GeoUtils.retrieve_new_google_map(latI, lonI, ZOOM - 1, WIDTH, HEIGHT)


        self.update_waypoints()

        # Prepare the image to be showed
        image = self.imageMetadata["bytes"]
        cvRGBImg = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        self.cvImage = cvRGBImg.copy()
        self.cvImageShadow = cvRGBImg.copy()
        self.im_to_show = cvRGBImg.copy()
        self.set_waypoints(self.wayPoints_lat_lon, current=False)
        self.refreshImage()


    def update_waypoints(self):
        self.wayPoints = []
        bbox = self.imageMetadata["bbox"]
        print(bbox)
        imagesize = self.imageMetadata["size"]
        list_wp = []
        for i in range(len(self.wayPoints_lat_lon)):
            geo_point = self.wayPoints_lat_lon[i]
            point = ImageUtils.posCoords2Image(bbox[0], bbox[1], bbox[2], bbox[3],
                                               geo_point[0], geo_point[1], imagesize[0], imagesize[1])

            self.wayPoints.append(point)

class MyDialog(QDialog):
    def __init__(self, parent=None):
        super(MyDialog, self).__init__(parent)

        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel|QDialogButtonBox.Ok)

        self.label_lat = QLabel("Introduce Latitude:")
        self.input_lat = QLineEdit()

        self.label_lon = QLabel("Introduce Longitude:")
        self.input_lon = QLineEdit()
        self.label_alt = QLabel("Introduce Altitude:")
        self.input_alt = QLineEdit()

        self.grid_layout = QGridLayout(self)
        self.grid_layout.addWidget(self.label_lat,0,0)
        self.grid_layout.addWidget(self.input_lat,0,1)
        self.grid_layout.addWidget(self.label_lon,1,0)
        self.grid_layout.addWidget(self.input_lon,1,1)
        self.grid_layout.addWidget(self.label_alt,2,0)
        self.grid_layout.addWidget(self.input_alt,2,1)
        self.grid_layout.addWidget(self.buttonBox,3,0)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

    def get_waypoint_to_add(self):
        self.accept()
        return self.input_lat.text(), self.input_lon.text(), self.input_alt.text()

    def accept(self):
        return self.input_lat.text(), self.input_lon.text(), self.input_alt.text()
