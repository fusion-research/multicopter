from __future__ import division

from twisted.internet import defer, reactor

import deferral
import xbee


@defer.inlineCallbacks
def main():
    xb = yield xbee.XBee(reactor, '/dev/ttyUSB0', 115200)
    
    while True:
        packet, = yield xb.packet_received.get_deferred()
        print packet
deferral.launch_main(main)
