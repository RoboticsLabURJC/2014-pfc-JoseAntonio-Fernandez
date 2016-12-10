
from PyQt5.QtWidgets import *
from MapClient.qtTest import Form
from owslib.wms import WebMapService
from MapClient.tools import GeoUtils


wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')
bbox = GeoUtils.getBoundingBox(-3.708283, 40.4153774, 500) # plaza mayor
img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                 styles=['default'],
                 srs='EPSG:4326',
                 bbox=bbox,
                 size=(500, 500),
                 format='image/png',
                 transparent=True)

out = open('plazaMayor500x500.png', 'wb')
out.write(img.read())
out.close()

if __name__ == '__main__':
    import sys

    app = QApplication(sys.argv)

    screen = Form()
    screen.show()

    sys.exit(app.exec_())

#TODO ¿prescindo de owslib?



#TODO pintar una señal en el medio
#TODO ver como muestro el recorrido de la aeronave
#TODO Interfaz grafico
#TODO Ver actitud, mapa y posteriormente imagen
#TODO Marcar en un mapa waypoints para generar un path a la aeronave
