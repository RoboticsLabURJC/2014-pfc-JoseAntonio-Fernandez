'''
Utility functions to operate with coordinates
'''


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

# a partir de una latitud y longitud obtener el encuadre para que quede en medio
#TODO Mejorar con la fórmula exacta y con los metros que deseemos
