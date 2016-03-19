from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=.01)

p = protocol.Protocol(s)
reader = p.read_packet()

l = None
while True:
    x = reader.next()
    if x is None: continue
    t = x[3] + 2**8*x[4] + 2**16*x[5] + 2**24*x[6]
    print x,
    if l is not None:
        print t-l-24500000
    else:
        print
    l = t
