
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from MapClient.tools import ImageUtils, GeoUtils

class Form(QWidget):
    def __init__(self, imageInput, parent=None):
        super(Form, self).__init__(parent)

        #Set the image Metadata
        self.imageMetadata = imageInput

        #Prepare the image to be showed
        self.image2show = ImageUtils.prepareInitialImage(self.imageMetadata["bytes"])
        pixmap = QPixmap()
        pixmap.loadFromData(self.image2show)

        self.imageLabel = QLabel()
        self.imageLabel.setPixmap(pixmap)
        self.imageLabel.setMouseTracking(True)
        self.imageLabel.mouseMoveEvent = self.getPos
        self.imageLabel.mousePressEvent= self.addWayPoint

        #self.labelXY = QLabel()
        self.labelGP = QLabel()
        self.table = QTableWidget()
        self.table.setColumnCount(0)
        self.table.setRowCount(0)
        self.table.insertColumn(0)

        itemCol = QTableWidgetItem("Waypoint")
        self.table.setHorizontalHeaderItem(0,itemCol)
        self.table.setMinimumSize(230,0)
        header = self.table.horizontalHeader()
        header.setStretchLastSection(True)

        layoutPpal = QGridLayout()
        layoutPpal.addWidget(self.imageLabel)
        layoutPpal.addWidget(self.table,0,1)
        #layoutPpal.addWidget(self.labelXY)
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

        latMin, lonMin, latMax, lonMax = self.imageMetadata['bbox']
        sizeX, sizeY = self.imageMetadata['size']
        lat, lon = ImageUtils.posImage2Coords(x, y, sizeX, sizeY, latMin, lonMin, latMax, lonMax)

        currentRowCount = self.table.rowCount()
        wayPoint = "lat: " + str(lat) + ' lon: ' +str(lon)
        item = QTableWidgetItem()
        item.setText(wayPoint)
        item.font()
        self.table.insertRow(currentRowCount)
        self.table.setItem(currentRowCount, 0, item)
        self.image2show = ImageUtils.addWayPointImg(self.image2show,x, y)

        #TODO ver si se puede hacer mejor
        pixmap = QPixmap()
        pixmap.loadFromData(self.image2show)
        self.imageLabel.setPixmap(pixmap)



#TODO ver como muestro el recorrido de la aeronave

#TODO Ver actitud, mapa y posteriormente imagen
