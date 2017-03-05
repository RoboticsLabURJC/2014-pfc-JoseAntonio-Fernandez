

from MapClient.tools import GeoUtils, ImageUtils
from cv2 import *

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

#img = open('images/plazaMayor600x600.png','rb')
bbox = GeoUtils.getBoundingBox(lat,lon, h)
#image = GeoUtils.retrieve_new_map(lat, lon, h, IMAGE_WIDTH, IMAGE_HEIGTH)

#image = {'bytes': img.read(), 'bbox': bbox, 'size': (IMAGE_WIDTH,IMAGE_HEIGTH)}
im = cv2.imread('images/plazaMayor600x600.png');
cv2.imshow('image',im)
