from __future__ import division

import random
import time

from twisted.internet import defer, reactor

import deferral
import xbee
import js


@defer.inlineCallbacks
def main():
    j = js.JS(reactor)
    xb = yield xbee.XBee(reactor, '/dev/ttyUSB0', 230400)
    
    while True:
        data = str(j.state.value[0][0].value)
        data = str(j.state.value[1][288])
        print data
        xb.transmit(data)
        yield deferral.sleep(.01 + .00015*len(data))
deferral.launch_main(main)
