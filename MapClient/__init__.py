import easyiceconfig as EasyIce
from parallelIce.cameraClient import CameraClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient

from MapClient.GUI.qtTest import MainGUI
from MapClient.GUI.threadGUI import ThreadGUI
from PyQt5.QtWidgets import QApplication

from MapClient.GUI.threadMap import ThreadMap
from MapClient.classes.navDataClient import NavDataClient
from MapClient.classes.MissionI import MissionI
from MapClient.tools import GeoUtils, ImageUtils
import cv2

IMAGE_WIDTH = 600
IMAGE_HEIGTH = 600

import signal

signal.signal(signal.SIGINT, signal.SIG_DFL)


lat = 40.333285
lon = -3.797859

'''
lat =-35.363261
lon =149.165230


lat = 40.333090
lon = -3.798212
'''


h = 0.5 # radius in kilometers

bbox = GeoUtils.getBoundingBox(lat,lon, h)
#im = GeoUtils.retrieve_new_map(lat, lon, h, IMAGE_WIDTH, IMAGE_HEIGTH)

'''
with open('images/tmp.png', 'wb') as f:
    f.write(im.read())


im = open('images/tmp.png','rb')
ImageUtils.prepareInitialImage(im.read(),IMAGE_WIDTH,IMAGE_HEIGTH)
'''
opencv_image = cv2.imread('images/imageWithDisclaimer.png',1);


image = {'bytes': opencv_image, 'bbox': bbox, 'size': (IMAGE_WIDTH,IMAGE_HEIGTH)}


if __name__ == '__main__':
    import sys

    ic = EasyIce.initialize(sys.argv)
    camera = CameraClient(ic, "UavViewer.Camera", True)
    navdata = NavDataClient(ic, "UavViewer.Navdata", True)
    pose = Pose3DClient(ic, "UavViewer.Pose3D", True)
    cmdvel = CMDVel(ic, "UavViewer.CMDVel")
    extra = Extra(ic, "UavViewer.Extra")
    mission = MissionI(ic, "UavViewer.Mission")

    app = QApplication(sys.argv)

    screen = MainGUI(image)
    screen.setFirstLocation(image)
    screen.setCamera(camera)
    screen.setPose3D(pose)
    screen.setNavData(navdata)
    screen.setCMDVel(cmdvel) # Not used yet
    screen.setExtra(extra)
    screen.setMission(mission)
    screen.show()


    t2 = ThreadGUI(screen)
    t2.daemon = True
    t2.start()

    t3 = ThreadMap(screen)
    t3.daemon = True
    t3.start()

    sys.exit(app.exec_())

#TODO ¿prescindo de owslib?




