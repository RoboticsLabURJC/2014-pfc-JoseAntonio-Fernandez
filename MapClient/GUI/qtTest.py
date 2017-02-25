import jderobot

from MapClient.GUI.sensorsWidget import SensorsWidget
from PyQt5 import QtCore
from PyQt5.QtCore import QSize
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import *

from MapClient.GUI.cameraWidget import CameraWidget
from MapClient.tools import ImageUtils, WayPoint

LEFT =1


class MainGUI(QWidget):
    updGUI = pyqtSignal()
    udpMap = pyqtSignal()


    def __init__(self, imageInput, parent=None):
        super(MainGUI, self).__init__(parent)

        self.updGUI.connect(self.updateGUI)
        #self.udpMap.connect(self.updatePosition())

        #Set the image Metadata
        self.imageMetadata = imageInput
        self.wayPoints = list

        #Prepare the image to be showed
        self.image2show = ImageUtils.prepareInitialImage(self.imageMetadata["bytes"], self.imageMetadata["size"][0],self.imageMetadata["size"][1])
        pixmap = QPixmap()
        pixmap.loadFromData(self.image2show)

        self.imageLabel = QLabel()
        self.imageLabel.setPixmap(pixmap)
        self.imageLabel.setMouseTracking(True)
        self.imageLabel.mouseMoveEvent = self.getPos
        self.imageLabel.mousePressEvent= self.addWayPoint

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
        self.planeFrame.setChecked(True)


        self.buttonLayout.addSpacing(210)
        self.buttonLayout.addWidget(self.toffButton)
        self.buttonLayout.addWidget(self.send2APM)
        self.buttonLayout.addWidget(self.cameraCheck)
        self.buttonLayout.addWidget(self.attitudeCheck)

        layoutPpal = QGridLayout()
        layoutPpal.addWidget(self.imageLabel)
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





    def getPos(self, event):
        '''
        function who reference the mouse position in the image over coordinades
        :param event: event executed
        :return: None
        '''
        x = event.pos().x()
        y = event.pos().y()
        latMin, lonMin, latMax, lonMax = self.imageMetadata['bbox']
        sizeX, sizeY = self.imageMetadata['size']
        lat, lon = ImageUtils.posImage2Coords(x, y, sizeX, sizeY, latMin, lonMin, latMax, lonMax)
        #self.labelXY.setText(str(x)+ " " + str(y))
        self.labelGP.setText("lat: " + str(lat) + ' lon: ' +str(lon))

    def addWayPoint(self, event):
        x = event.pos().x()
        y = event.pos().y()

        if (event.button() == QtCore.Qt.LeftButton):
            latMin, lonMin, latMax, lonMax = self.imageMetadata['bbox']
            sizeX, sizeY = self.imageMetadata['size']
            lat, lon = ImageUtils.posImage2Coords(x, y, sizeX, sizeY, latMin, lonMin, latMax, lonMax)

            currentRowCount = self.table.rowCount()
            wayPoint = WayPoint.WayPoint(x,y,lat,lon)
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
            self.image2show = ImageUtils.addWayPointImg(self.image2show,x, y, currentRowCount)
            #self.wayPoints.insert(wayPoint)

            #TODO ver si se puede hacer mejor
            pixmap = QPixmap()
            pixmap.loadFromData(self.image2show)
            self.imageLabel.setPixmap(pixmap)

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
        mission = jderobot.PoseSequence()
        mission = jderobot.MissionData()
        i = 0
        for row in range(colCount):
            pose = jderobot.Pose3DData
            text = self.table.item(row, 0).text()
            pos_lat = text.find("lat:")
            pos_lon = text.find("lon:")
            if pos_lat != -1:
                lat = (text[pos_lat + 4:pos_lon])
                pose.x = float(lat)
                print(lat)
                lon = (text[pos_lon + 4:])
                pose.y = float(lon)
                print(lon)
            else:
                if "LAND in " in text:
                    pos = text.find("LAND in ") + 8
                    position = text[pos:].split(" ")
                    lat = position[0]
                    lon = position[1]
                    pose.x = float(lat)
                    pose.y = float(lon)
                else:
                    pos = text.find("TAKE OFF to ") + 12
                    position = text[pos:].split(" ")
                    lat = position[0]
                    lon = position[1]
                    pose.x = float(lat)
                    pose.y = float(lon)

            alt = self.table.item(row, 1)
            pose.h = int(alt.text())
            mission[i] = pose
            i += 1

            print(text + alt.text())



        #send the mission
        #PoseTheading = threading.Thread(target=ice_init.sendWP, args=(pose,), name='Pose_Theading')
        #PoseTheading.daemon = True
        #PoseTheading.start()
        #ice_init.sendWP(pose)

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


    def updateGUI(self):
        '''
        Update the position of the UAV in the map and the attitude and camera image in it's own widgets
        :return:
        '''

        '''
        if not (self.pose.getPose3D() == None):
            data = self.pose.getPose3D()
            img_to_show = ImageUtils.refreshPosition(self.image2show,data.x, data.y)
            # TODO ver si se puede hacer mejor
            pixmap = QPixmap()
            pixmap.loadFromData(img_to_show)
            self.imageLabel.setPixmap(pixmap)
        '''
        # HARDCODED
        self.updateImage()

        # HARDCODED

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
        self.navdata.stop()
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


#TODO ver como muestro el recorrido de la aeronave
