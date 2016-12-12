
from PyQt5.QtWidgets import *
from MapClient.qtTest import Form
from owslib.wms import WebMapService
from MapClient.tools import GeoUtils, ImageUtils

IMAGE_WIDTH = 600
IMAGE_HEIGTH = 600



wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')
lat = 40.4153774
lon = -3.708283
h = 0.5 # radius in kilometers

bbox = GeoUtils.getBoundingBox(lat, lon, h) # plaza mayor

img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                 styles=['default'],
                 srs='EPSG:4326',
                 bbox=(bbox),
                 size=(IMAGE_WIDTH, IMAGE_HEIGTH),
                 format='image/png',
                 transparent=True)


#image = Image.open('images/plazaMayor500x500.png')

image = {'bytes': img.read(), 'bbox': bbox, 'size': (IMAGE_WIDTH,IMAGE_HEIGTH)}



if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)

    screen = Form(image)
    screen.show()

    sys.exit(app.exec_())

#TODO ¿prescindo de owslib?




