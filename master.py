from __future__ import division

import random
import time

from twisted.internet import defer, reactor

import deferral
import xbee


@defer.inlineCallbacks
def main():
    xb = yield xbee.XBee(reactor, '/dev/ttyUSB0', 230400)
    
    while True:
        LENGTH = 1
        k = str(random.randrange(10**LENGTH)).zfill(LENGTH)
        assert len(k) == LENGTH
        
        xb.transmit(k)
        send_time = time.time()
        
        try:
            packet, = yield deferral.wrap_timeout(xb.packet_received.get_deferred(), .2)
        except deferral.TimeoutError:
            print 'dropped'
            continue
        
        recv_time = time.time()
        
        assert packet['data'] == k[::-1]
        
        print recv_time - send_time
deferral.launch_main(main)
