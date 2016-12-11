from PIL import Image, ImageDraw, ImageFont
import io

def prepareInitialImage(arrByte):
    '''
    Prepare the image recieved from PNOA to be showed in the app
    :param arrByte: array of bytes of the image
    :return: URI of the saved image
    '''
    image = Image.open(io.BytesIO(arrByte))

    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype("fonts/OpenSans-Bold.ttf", 11)
    draw.text((420, 482), "PNOA cedido por © Instituto Geográfico Nacional de España", font=font, fill="white")
    draw.line((385, 240, 385, 260), fill=255, width=2)
    draw.line((375, 250, 395, 250), fill=255, width=2)

    image.save("images/imageWithDisclaimer.png")
    return "images/imageWithDisclaimer.png"

def refreshPosition(arrByte, x, y):
    None
    #TODO implementar función que a partir de unos pixeles pinte la cruceta

def addWayPoint(arrByte, x, y):
    None
    # TODO implementar función que a partir de unos pixeles pinte la cruceta de otro color en esa posición