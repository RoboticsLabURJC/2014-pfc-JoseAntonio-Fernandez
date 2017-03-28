from PyQt5 import QtCore
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
import numpy as np
import cv2
import math,unicodedata

from cv2 import cornerEigenValsAndVecs

DISCLAIMER_WIDTH = 345
DISCLAIMER_HEIGHT = 20

class ImageGUI(QWidget):




    def __init__(self, imageInput, im, parent=None):
        super(ImageGUI, self).__init__(parent)

        self.imageLabel = QLabel()
        self.imgLabel  = QLabel()
        self.imLabel  = QLabel()

        self.wp = [[100, 100], [450, 100], [45, 450]]


        cvRGBImg = cv2.cvtColor(imageInput,cv2.COLOR_RGB2BGR)
        height, width, channel = cvRGBImg.shape
        bytesPerLine = 3 * width
        self.set_disclaimer(cvRGBImg)
        self.cvImage = cvRGBImg.copy()

        self.cvImageShadow = cvRGBImg.copy()
        self.im_to_show = cvRGBImg.copy()

        qimg = QImage(cvRGBImg.data, width, height, bytesPerLine, QImage.Format_RGB888)
        qpm = QPixmap.fromImage(qimg)
        self.imageLabel.setPixmap(qpm)

        '''
        image = QImage(imageInput.data, imageInput.shape[1], imageInput.shape[0], imageInput.shape[1] * imageInput.shape[2], QImage.Format_RGB888);
        self.imgLabel.setPixmap(QPixmap.fromImage(image))

        pixmap = QPixmap()
        pixmap.loadFromData(im)
        self.imLabel.setPixmap(pixmap)
        '''

        self.layoutPpal = QGridLayout()
        self.layoutPpal.addWidget(self.imageLabel,0,0)
        #self.layoutPpal.addWidget(self.imgLabel, 0, 1)
        #self.layoutPpal.addWidget(self.imLabel, 0, 2)

        self.setLayout(self.layoutPpal)
        self.setWindowTitle("AUVCommander")




    def setPosition(self, x, y,angle):
        self.refresh_shadow(x,y)
        self.set_waypoints(self.wp)
        self.draw_triangle(x, y, angle)
        self.refreshImage()


    def set_waypoints(self, wayPoints):
        n=0
        pts = np.array(wayPoints, np.int32)
        pts = pts.reshape((-1, 1, 2))
        cv2.polylines(self.cvImageShadow, [pts], False, (250, 250, 250), thickness=1)
        for waypoint in wayPoints:
            n+=1
            s = str(n)
            cv2.circle(self.cvImageShadow, (waypoint[0], waypoint[1]), 3, [0, 0, 255], thickness=-1, lineType=8, shift=0)
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
        theta = math.radians(angle)
        rotatedPolygon = []
        for corner in polygon:
            rotatedPolygon.append((corner[0] * math.cos(theta) - corner[1] * math.sin(theta),
                                   corner[0] * math.sin(theta) + corner[1] * math.cos(theta)))
        return rotatedPolygon

    def draw_triangle(self, x, y, angle):
        triangle = [[x-6,y-3],[x,y+4],[x+6,y-4]]
        print('triangle points' + str(triangle))
        center = self.center_of_triangle(triangle)
        print('center' + str(center))
        triangleCartesian = self.change_coordinate_system(triangle, center,True)
        print('triangle points in center' + str(triangleCartesian))
        rotated_triangleCartesian = self.rotate_polygon(triangleCartesian,angle)
        image_center = [x,y]
        rotated_triangle = self.change_coordinate_system(rotated_triangleCartesian,image_center ,False)
        print(rotated_triangle)
        pts = np.array(rotated_triangle, np.int32)
        pts = pts.reshape((-1,1,2))
        self.im_to_show = self.cvImageShadow.copy()
        cv2.fillPoly(self.im_to_show, [pts], (250,0,0))
        cv2.polylines(self.im_to_show,[pts],True,(250,0,0),thickness=1)

    def refresh_shadow(self,x,y):
        cv2.circle(self.cvImageShadow, (x, y), 1, [50, 50, 50], thickness=-1, lineType=8, shift=0)

    def center_of_triangle(self, triangle):
        center_x = (triangle[0][0] + triangle[1][0] + triangle[2][0]) / 3;
        center_y = (triangle[0][1]+ triangle[1][1] + triangle[2][1]) / 3;
        return [center_x,center_y]