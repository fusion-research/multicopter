from __future__ import division

import random
import time
import struct

from twisted.internet import defer, reactor, endpoints

import deferral
import xbee
import js
import sdgps

@defer.inlineCallbacks
def main():
    j = js.JS(reactor)
    sdgps_protocol = yield endpoints.TCP4ClientEndpoint(reactor, "localhost", 1234).connect(deferral.AutoServerFactory(lambda addr: sdgps.SDGPSProtocol()))
    xb = yield xbee.XBee(reactor, '/dev/ttyUSB0', 230400)
    
    while True:
        #for k in j.state.value[0]:
        #    print k, j.state.value[0][k].value
        #print j.state.value[1]
        
        if sdgps_protocol.last_imu is not None:
            print sdgps_protocol.last_imu.timestamp
            w = (
                sdgps_protocol.last_imu.angular_velocity[1],
                -sdgps_protocol.last_imu.angular_velocity[0],
                sdgps_protocol.last_imu.angular_velocity[2],
            )
            print w
        
        if not j.state.value[1][288] or sdgps_protocol.last_imu is None:
            stop = 1
            z, tx, ty, tz = 0, 0, 0, 0
        else:
            stop = 0
            
            dwx = (j.state.value[0][0].value-550)/(842-550) * 3
            dwy = -(j.state.value[0][1].value-511)/(873-511) * 3
            dwz = -(j.state.value[0][5].value-130)/(210-130) * 3
            tx = 0.5 * (dwx - w[0])
            ty = 0.5 * (dwy - w[1])
            tz = 0.5 * (dwz - w[2])
            print dwx, dwy, dwz
            print tx, ty, tz
            print reactor.seconds()
            print
            z = (j.state.value[0][6].value-189)/(42-189) * (4.6*6)
        
        data = struct.pack('<B4f', stop, z, tx, ty, tz)
        xb.transmit(data)
        yield deferral.sleep(.01 + .00015*len(data))
deferral.launch_main(main)
