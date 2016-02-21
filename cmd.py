from __future__ import division

import struct
import time
import random

import serial


s = serial.Serial('/dev/ttyUSB0', 115200, timeout=.3)

for i in xrange(1, 128, 2):
    print i
    s.write(chr(i))
    time.sleep(.2)

last_revs = None
last_t = None
while True:
    t = time.time()
    s.write(chr(0))
    res = s.read(2)
    if len(res) == 2 and res[0] == '\0' and ord(res[1]) & 0x80:
        revs = ord(res[1]) & ~0x80
        
        if last_revs is not None:
            delta_revs = (revs - last_revs) % 128
            delta_t = t - last_t
            print delta_revs, delta_revs/delta_t
        last_revs = revs
        last_t = t
    time.sleep(.1)
