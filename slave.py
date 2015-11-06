from __future__ import division

import time

from twisted.internet import defer, reactor

import deferral
import xbee


@defer.inlineCallbacks
def main():
    xb = yield xbee.XBee(reactor, '/dev/ttyO0', 115200)
    
    while True:
        xb.transmit('hello' + str(time.time()))
        yield deferral.sleep(1)
deferral.launch_main(main)
