from MapClient.tools import GeoUtils

IMAGE_WIDTH = 600
IMAGE_HEIGTH = 600

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


lat = 40.4153774
lon = -3.708283

'''
lat =-35.363261
lon =149.165230


lat = 40.333090
lon = -3.798212
'''


h = 0.3 # radius in kilometers



bbox = GeoUtils.getBoundingBox(lat,lon, h)
image = GeoUtils.retrieve_new_map(lat, lon, h, IMAGE_WIDTH, IMAGE_HEIGTH)