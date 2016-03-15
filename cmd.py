from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/ttyUSB0', 115200)
id_ = int(sys.argv[1])

p = protocol.Protocol(s)

p.write_packet(struct.pack('>BBB', 2, id_, 93))
for x in p.read_packet(): print x
fasdfa

for i in xrange(50, 100):
    print i
    s.write('3719153c'.decode('hex') + chr(i))
    time.sleep(.05)
import math
import itertools
for i in itertools.count():
    power = 175+75*math.sin(i*.05 * 10)
    print power
    s.write('3719153c'.decode('hex') + chr(int(round(power))))
    time.sleep(.05)

last_revs = None
last_t = None
while True:
    t = time.time()
    s.write('3719153c'.decode('hex') + chr(0))
    revs = ord(read_packet(1))
    if last_revs is not None:
        delta_revs = (revs - last_revs) % 256
        delta_t = t - last_t
        print delta_revs, delta_revs/delta_t
    last_revs = revs
    last_t = t
    time.sleep(.3)
