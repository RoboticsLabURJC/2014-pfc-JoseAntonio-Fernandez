
from owslib.wms import WebMapService



def getBoundingBox(lat, lon, distance):
    '''
    version 0.1
    Function who retrieves the min and max point of a square where the central point is informed
    :param lat: latitude on decimal
    :param lon: longitude on decimal
    :param distance: distance on meters from point to the vertex NOT Used yet
    :return: bounding points in decimal [0.0,0.0,0.0,0.0]
    '''
    normlat = (distance * 0.0000094413)
    normlon = (distance * 0.0000061250)
    return (round(lat-normlat,7), round(lon-normlon,7), round(lat+normlat,7), round(lon+normlon,7))


wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')
bbox = getBoundingBox(-3.708283, 40.4153774, 500) # plaza mayor
img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                 styles=['default'],
                 srs='EPSG:4326',
                 bbox=bbox,  # Puerta de Alcalá
                 size=(500, 500),
                 format='image/png',
                 transparent=True)

print(img.read())
out = open('plazaMayor500x500.png', 'xb')
y = bytes()
y = img.read()
out.write(y)
out.close()







#TODO a partir de una latitud y longitud obtener el encuadre para que quede en medio v0 hecho
#TODO pintar una señal en el medio
#TODO ver como muestro el recorrido de la aeronave
#TODO Interfaz grafico
#TODO Ver actitud, mapa y posteriormente imagen
#TODO Marcar en un mapa waypoints para generar un path a la aeronave
#TODO ¿prescindo de owslib?