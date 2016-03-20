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
    for id_ in [3]:
        p.write_packet(struct.pack('<BBBH', 2, id_, 2, i))
    time.sleep(.01)
