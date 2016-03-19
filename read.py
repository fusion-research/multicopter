from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=.01)

p = protocol.Protocol(s)
reader = p.read_packet()

l = 0
while True:
    x = reader.next()
    if x is None: continue
    t = x[3] + 2**8*x[4] + 2**16*x[5] + 2**24*x[6]
    t2 = x[3+4] + 2**8*x[4+4] + 2**16*x[5+4] + 2**24*x[6+4]
    print x, t-l, t2-t
    l = t
