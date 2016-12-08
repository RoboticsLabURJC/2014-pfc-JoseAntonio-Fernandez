from owslib.wms import WebMapService

wms = WebMapService('http://www.ign.es/wms-inspire/pnoa-ma', version='1.3.0',timeout=100)

img = wms.getmap(layers=['OI.MosaicElement'],
                 styles=['default'],
                 srs='EPSG:25829',
                 bbox=(-34.0, 22.0, 67.0, 52.0),
                 size=(300, 250),
                 format='image/jpeg',
                 transparent=True)

out = open('jpl_mosaic_visb.jpg', 'wb')
out.write(img.read())
out.close()