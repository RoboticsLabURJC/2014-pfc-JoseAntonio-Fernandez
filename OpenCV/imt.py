from encodings import shift_jis

import cv2
from PyQt5.QtWidgets import QApplication

from OpenCV.ImageLabel import ImageGUI
from OpenCV.threadGUI import ThreadGUI
from MapClient import *

import threading


IMAGE_WIDTH = 600
IMAGE_HEIGTH = 600

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


lat = 40.4153774
lon = -3.708283

'''
lat =-35.363261
lon =149.165230

cv2.imshow('image',img)
lat = 40.333090
lon = -3.798212
'''


h = 0.3 # radius in kilometers


'''
with open('images/plazaMayor600x600.png', 'wb') as f:
    f.write(img.read())
'''

img = open('images/plazaMayor600x600.png','rb')
bbox = GeoUtils.getBoundingBox(lat,lon, h)
result = GeoUtils.retrieve_new_google_map(lat, lon, h, IMAGE_WIDTH, IMAGE_HEIGTH)

with open('images/tmp.png', 'wb') as f:
    f.write(result.read())

opencv_image = cv2.imread('images/tmp.png',1);

cv2.imshow("image", opencv_image)
#image = {'bytes': img.read(), 'bbox': bbox, 'size': (IMAGE_WIDTH,IMAGE_HEIGTH)}
#print ("openning " + 'images/plazaMayor600x600.png')
#im = cv2.imread('images/plazaMayor600x600.png',1);




'''

if __name__ == '__main__':
    import sys


    app = QApplication(sys.argv)
    screen = ImageGUI(im,img.read())
    screen.show()

    t2 = ThreadGUI(screen)
    t2.daemon = True
    t2.start()

    sys.exit(app.exec_())
'''

