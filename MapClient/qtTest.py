
from PyQt5.QtWidgets import *
from PyQt5 import QtCore
from PyQt5.QtGui import QPixmap
from MapClient.tools import ImageUtils, GeoUtils, WayPoint
from MapClient.ice import ice_init
import jderobot
import threading

LEFT =1


class Form(QWidget):
    def __init__(self, imageInput, parent=None):
        super(Form, self).__init__(parent)

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
        self.attitudeCheck = QCheckBox("Attitude")

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
        PoseTheading = threading.Thread(target=ice_init.sendWP, args=(pose,), name='Pose_Theading')
        PoseTheading.daemon = True
        PoseTheading.start()
        #ice_init.sendWP(pose)

#TODO ver como muestro el recorrido de la aeronave

#TODO Ver actitud, mapa y posteriormente imagen
