
from math import *
from PIL import Image
from owslib.wms import WebMapService
import numpy as np
import io

'''
Utility functions to operate with coordinates
'''


MIN_LAT = radians(-90)
MAX_LAT = radians(90)
MIN_LON = radians(-180)
MAX_LON = radians(180)
EARTH_RADIUS_WGS84 = 6371  # kilometers based on the ellipsoid of the standard WGS84 used in the IGN EPSG:4326



def getBoundingBox(lat, lon, distance):
    '''
    version 0.2
    Function who retrieves the min and max point of a square where the central point is informed.
    Based in the solution in Java of Jan Philip Matuschek in http://janmatuschek.de/LatitudeLongitudeBoundingCoordinates
    adapted to Python for Jeremy Fein in https://github.com/jfein/PyGeoTools/blob/master/geolocation.py with some adaptions
    like the EARTH_RADIUS adapted to WGS84

    :param lat: latitude on decimal
    :param lon: longitude on decimal
    :param distance: distance on meters from point to the corners
    :return: bounding points in decimal [0.0,0.0,0.0,0.0]
    '''

    # to radians
    radValues = from_degrees(lat, lon)
    rad_dist = distance / EARTH_RADIUS_WGS84

    min_lat = radValues[0] - rad_dist
    max_lat = radValues[0] + rad_dist

    if min_lat > MIN_LAT and max_lat < MAX_LAT:
        delta_lon = asin(sin(rad_dist) / cos(radValues[1]))

        min_lon = radValues[1] - delta_lon
        if min_lon < MIN_LON:
            min_lon += 2 * pi

        max_lon = radValues[1] + delta_lon
        if max_lon > MAX_LON:
            max_lon -= 2 * pi
    # a pole is within the distance, The up code isn't accurate in these cases
    else:
        min_lat = max(min_lat, MIN_LAT)
        max_lat = min(max_lat, MAX_LAT)
        min_lon = MIN_LON
        max_lon = MAX_LON

    # back to degrees
    southWestPoint = from_radians(min_lat, min_lon)
    northEastpoint = from_radians(max_lat, max_lon)
    return southWestPoint[1], southWestPoint[0], northEastpoint[1], northEastpoint[0]


def from_degrees(deg_lat, deg_lon):
    '''
    Converts from degrees to radians
    Based in the solution in Java of Jan Philip Matuschek in http://janmatuschek.de/LatitudeLongitudeBoundingCoordinates
    :param deg_lat: latitude in degrees
    :param deg_lon: longitude in degrees
    :return: latitude, longitude in radians
    '''

    rad_lat = radians(deg_lat)
    rad_lon = radians(deg_lon)
    return (rad_lat, rad_lon)


def from_radians(rad_lat, rad_lon):
    '''
    Converts from radians to degrees
    Based in the solution in Java of Jan Philip Matuschek in http://janmatuschek.de/LatitudeLongitudeBoundingCoordinates
    :param rad_lat: latitude in radians
    :param rad_lon: longitude in radians
    :return: latitude, longitude in degrees
    '''
    deg_lat = degrees(rad_lat)
    deg_lon = degrees(rad_lon)
    return (deg_lat, deg_lon)


def distance_to(valores, other ):
    '''
    Based in the solution in Java of Jan Philip Matuschek in http://janmatuschek.de/LatitudeLongitudeBoundingCoordinates
    Calculate the distance between 2 points in the earth
    :param valores:
    :param other:
    :return:
    '''
    radius = EARTH_RADIUS_WGS84
    radValuesIni = from_degrees(valores[0], valores[1])
    radValuesOther = from_degrees(other[0], other[1])

    return radius * acos(
        sin(radValuesIni[0]) * sin(radValuesOther[0]) +
        cos(radValuesIni[0]) *
        cos(radValuesOther[0]) *
        cos(radValuesIni[1] - radValuesOther[1])
    )


def retrieve_new_map(lat, lon, radius, width, heigth):

    wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')
    bbox = getBoundingBox(lat, lon, radius)

    img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                     styles=['default'],
                     srs='EPSG:4326',
                     bbox=(bbox),
                     size=(width, heigth),
                     format='image/png',
                     transparent=True)

    image = Image.open(io.BytesIO(img.read()))
    image.load()

    numpy_image = np.asarray(image.getdata(), dtype='uint8')


    return {'numpy' : numpy_image, 'bytes': img.read(), 'bbox': bbox, 'size': (width,heigth)}