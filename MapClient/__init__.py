from owslib.wms import WebMapService

wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0',timeout=100)

img = wms.getmap(layers=['OI.MosaicElement'],
                 styles=['default'],
                 srs='EPSG:25829',
                 bbox=(-2.0, 2.0, 6.0, 5.0),
                 size=(300, 250),
                 format='image/jpeg',
                 transparent=True)

print(img.read())
out = open('jpl_mosaic_visb1.jpg', 'xb')
y = bytes()
y = img.read()
out.write(y)
out.close()