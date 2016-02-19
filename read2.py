from __future__ import division

import struct
import time
import random

import serial


s = serial.Serial('/dev/ttyUSB0', 115200)

s.write('g')

while True:
    r = ord(s.read(1))
    if r != 42: continue
    speed = struct.unpack('>H', s.read(2))
    print speed
