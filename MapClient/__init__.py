
from PyQt5.QtWidgets import *
from MapClient.qtTest import Form
from owslib.wms import WebMapService
from MapClient.tools import GeoUtils, ImageUtils


wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')
bbox = GeoUtils.getBoundingBox(-3.708283, 40.4153774, 500) # plaza mayor
img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                 styles=['default'],
                 srs='EPSG:4326',
                 bbox=(bbox),
                 size=(770, 500),
                 format='image/png',
                 transparent=True)


#image = Image.open('images/plazaMayor500x500.png')

image = {'bytes': img.read(), 'bbox': bbox, 'size': (770,500)}



if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)

    screen = Form(image)
    screen.show()

    sys.exit(app.exec_())

#TODO ¿prescindo de owslib?




