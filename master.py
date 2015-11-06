from __future__ import division

import random
import time

from twisted.internet import defer, reactor

import deferral
import xbee


@defer.inlineCallbacks
def main():
    xb = yield xbee.XBee(reactor, '/dev/ttyUSB0', 115200)
    
    in_flight = {}
    
    def cb(packet):
        print time.time() - in_flight.pop(packet['data'][::-1])
    xb.packet_received.watch(cb)
    
    while True:
        k = str(random.randrange(2**8))
        
        xb.transmit(k)
        in_flight[k] = time.time()
        
        yield deferral.sleep(.1)
deferral.launch_main(main)
