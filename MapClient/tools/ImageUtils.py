from PIL import Image, ImageDraw, ImageFont
import io

DISCLAIMER_WIDTH = 345
DISCLAIMER_HEIGHT = 20

def prepareInitialImage(arrByte,width , height):
    '''
    Prepare the image recieved from PNOA to be showed in the app
    :param arrByte: array of bytes of the image
    :return: URI of the saved image
    '''
    image = Image.open(io.BytesIO(arrByte))

    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("fonts/OpenSans-Bold.ttf", 11)
    draw.text((width-DISCLAIMER_WIDTH, height-DISCLAIMER_HEIGHT), "PNOA cedido por © Instituto Geográfico Nacional de España", font=font, fill="white")
    imgByteArr = io.BytesIO()
    image.save(imgByteArr, format='PNG')
    imgByteArr = imgByteArr.getvalue()
    image.save("images/imageWithDisclaimer.png")
    return imgByteArr


def refreshPosition(arrByte, x, y):
    None
    #TODO implementar función que a partir de unos pixeles pinte la cruceta

def addWayPointImg(arrByte, x, y):
    image = Image.open(io.BytesIO(arrByte))
    draw = ImageDraw.Draw(image)
    draw.line((x-10, y, x+10, y), fill=(255,0,0), width=2)
    draw.line((x, y-10, x, y+10), fill=(255,0,0), width=2)
    imgByteArr = io.BytesIO()
    image.save(imgByteArr, format='PNG')
    imgByteArr = imgByteArr.getvalue()
    return imgByteArr

def posImage2Coords(x, y, tamImageX, tamImageY, latMin, lonMin, latMax, lonMax):
    '''
    Obtains the coords referenced at a position over the image
    :param x: image x position
    :param y: image y position
    :param tamImageX: width of the image
    :param tamImageY: height of the image
    :param latMin: latitude on the y axis at the bottom left corner
    :param lonMin: longitude on the x axis at the bottom left corner
    :param latMax: latitude on the y axis at the upper right corner
    :param lonMax: longitude on the x axis at the upper right corner
    :return: lat, lon who correspond to the point in the image
    '''
    distCoordY = round(latMax - latMin,7)
    distCoordX = round(lonMax - lonMin, 7)

    lat = latMin + (y * (distCoordY/tamImageY))
    lon = lonMin + (x * (distCoordX / tamImageX))
    return round(lat,7), round(lon,7)