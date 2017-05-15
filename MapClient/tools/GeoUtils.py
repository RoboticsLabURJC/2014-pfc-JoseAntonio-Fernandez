
from math import *
from PIL import Image
from owslib.wms import WebMapService
import numpy as np
import io
from urllib.request  import urlopen
from MapClient.tools import ImageUtils
import cv2


'''
Utility functions to operate with coordinates
'''


MIN_LAT = radians(-90)
MAX_LAT = radians(90)
MIN_LON = radians(-180)
MAX_LON = radians(180)
H = 0.5
MERCATOR_RANGE = 256.0
EARTH_RADIUS_WGS84 = 6371.0  # kilometers based on the ellipsoid of the standard WGS84 used in the IGN EPSG:4326
DEAD_BAND = 10 # 10 percent


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

def center_of_triangle(triangle):
        center_x = (triangle[0][0] + triangle[1][0] + triangle[2][0]) / 3;
        center_y = (triangle[0][1]+ triangle[1][1] + triangle[2][1]) / 3;
        return [center_x,center_y]

def change_coordinate_system(points, origin, toCartsian=True):
        points_transformed = []
        if toCartsian:
            for corner in points:
                x = corner[0] - origin[0]
                y = origin[1] - corner[1]
                points_transformed.append([x, y])
        else:
            for corner in points:
                x = corner[0] + origin[0]
                y = origin[1] + corner[1]
                points_transformed.append([x, y])

        return points_transformed

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

    bbox = getBoundingBox(lat, lon, radius)

    wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0')
    bbox = getBoundingBox(lat, lon, radius)
    print(bbox)
    img = wms.getmap(layers=['OI.OrthoimageCoverage'],
                     styles=['default'],
                     srs='EPSG:4326',
                     bbox=(bbox),
                     size=(width, heigth),
                     format='image/png',
                     transparent=True)

    with open('images/tmp.png', 'wb') as f:
        f.write(img.read())

    ImageUtils.prepareInitialImage(img.read(), width, heigth)
    opencv_image = cv2.imread("images/imageWithDisclaimer.png", 1)

    image = {'bytes': opencv_image, 'bbox': bbox, 'size': (width, heigth)}
    return image

def retrieve_new_google_map(lat, lon, zoomInt, width, heigth):

    url = "http://maps.googleapis.com/maps/api/staticmap?"
    center = "center=" + str(lat) + "," + str(lon)
    size = "size=" + str(int(width)) + "x" + str(int(heigth))
    zoom = "zoom=" + str(zoomInt)
    type = "maptype=satellite"
    sensor = "sensor=true"
    scale = "scale=1"
    url = url + center + "&" + size + "&" + zoom + "&" + type + "&" + sensor + "&" + scale

    print(url)
    result = urlopen(url=url)
    if result.getcode() != 200:
        print ("Error | AUVCommander: Error retrieving new map from Google " + result.getcode())
        sys.exit(1)

    with open('images/tmp.png', 'wb') as f:
        f.write(result.read())

    opencv_image = cv2.imread("images/tmp.png", 1)
    bbox = MercatorProjection.getCorners((lat, lon), zoomInt, width, heigth)
    print(bbox)
    image = {'bytes': opencv_image, 'bbox': bbox, 'size': (width, heigth)}
    return image


def is_position_close_border(lat, lon , bbox):
    dif_lat = bbox[3] - bbox[1]
    dif_lon = bbox[2] - bbox[0]
    dead_lat = 10 * dif_lat / 100.0
    dead_lon = 10 * dif_lon / 100.0
    if(((lat < (bbox[1] + dead_lat)) or (lat > (bbox[3] - dead_lat))) or
               ((lon < (bbox[0] + dead_lon)) or (lon > (bbox[2] - dead_lon)))):
                return True
    else:
        return False

def bound(value, opt_min, opt_max):
    if (opt_min != None):
        value = max(value, opt_min)
    if (opt_max != None):
        value = min(value, opt_max)
    return value


def degreesToRadians(deg):
    return deg * (pi / 180)


def radiansToDegrees(rad):
    return rad / (pi / 180)



class MercatorProjection:
    def __init__(self):
        self.pixelOrigin_ = (MERCATOR_RANGE / 2, MERCATOR_RANGE / 2)
        self.pixelsPerLonDegree_ = MERCATOR_RANGE / 360
        self.pixelsPerLonRadian_ = MERCATOR_RANGE / (2 * pi)

    def fromLatLngToPoint(self, latLng, opt_point=None):
        point =(0,0)
        if opt_point is not None:
            point = opt_point
        origin = self.pixelOrigin_
        point = list(point)
        point[0] = origin[0] + latLng[1] * self.pixelsPerLonDegree_
        # NOTE(appleton): Truncating to 0.9999 effectively limits latitude to
        # 89.189.  This is about a third of a tile past the edge of the world tile.
        siny = bound(sin(degreesToRadians(latLng[0])), -0.9999, 0.9999)
        point[1] = origin[1] + 0.5 * log((1 + siny) / (1 - siny)) * -     self.pixelsPerLonRadian_
        return point


    def fromPointToLatLng(self, point):
        origin = self.pixelOrigin_
        lng = (point[0] - origin[0]) / self.pixelsPerLonDegree_
        latRadians = (point[1] - origin[1]) / -self.pixelsPerLonRadian_
        lat = radiansToDegrees(2 * atan(exp(latRadians)) - pi / 2)
        return (lat, lng)


# pixelCoordinate = worldCoordinate * pow(2,zoomLevel)

    def getCorners(center, zoom, mapWidth, mapHeight):
        scale = 2 ** zoom
        proj = MercatorProjection()
        centerPx = proj.fromLatLngToPoint(center)
        SWPoint = (centerPx[0] - (mapWidth / 2) / scale, centerPx[1] + (mapHeight / 2) / scale)
        SWLatLon = proj.fromPointToLatLng(SWPoint)
        NEPoint = (centerPx[0] + (mapWidth / 2) / scale, centerPx[1] - (mapHeight / 2) / scale)
        NELatLon = proj.fromPointToLatLng(NEPoint)
        return SWLatLon[1], SWLatLon[0], NELatLon[1], NELatLon[0]