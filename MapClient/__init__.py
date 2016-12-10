
from owslib.wms import WebMapService

wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')

img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                 styles=['default'],
                 srs='EPSG:4326',
                 bbox=(-3.6934474484697,40.416876952149,-3.6840060727373,40.423002044655),#Puerta de Alcalá
                 size=(300, 250),
                 format='image/png',
                 transparent=True)

print(img.read())
out = open('exampleThatWorks.png', 'xb')
y = bytes()
y = img.read()
out.write(y)
out.close()

#TODO a partir de una latitud y longitud obtener el encuadre para que quede en medio
#TODO pintar una señal en el medio
#TODO ver como muestro el recorrido de la aeronave
#TODO Interfaz grafico
#TODO Ver actitud, mapa y posteriormente imagen
#TODO Marcar en un mapa waypoints para generar un path a la aeronave