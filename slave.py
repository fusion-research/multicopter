from __future__ import division

import time

import xbee


xb = xbee.XBee('/dev/ttyO0', 115200)

while True:
    xb.transmit('hello' + str(time.time()))
    time.sleep(1)
