from __future__ import division

import struct
import time
import random

import serial


s = serial.Serial('/dev/ttyUSB0', 115200)

s.write('g')

while ord(s.read(1)) != 42: pass

while True:
    x = struct.unpack('>6B', s.read(6))
    print ' '.join('{0:08b}'.format(y) for y in x)
