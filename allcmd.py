from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/ttyUSB0', 115200)

p = protocol.Protocol(s)

for i in xrange(200, 4096):
    print i
    dev = protocol.Device(p, 2, id_)
    for id_ in [3]:
        dev.write_packet(struct.pack('<BH', 2, i))
    time.sleep(.01)
