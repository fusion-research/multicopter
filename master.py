from __future__ import division

import xbee


xb = xbee.XBee('/dev/ttyUSB0', 115200)

while True:
    print xb.read()
