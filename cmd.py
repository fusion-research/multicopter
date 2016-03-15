from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/ttyUSB0', 115200)
id_ = int(sys.argv[1])

p = protocol.Protocol(s)

for i in xrange(50, 100):
    print i
    p.write_packet(struct.pack('>BBBB', 2, id_, 2, i))
    time.sleep(.05)

if 0:
    import math
    import itertools
    for i in itertools.count():
        power = 175+75*math.sin(i*.05 * 10)
        print power
        p.write_packet(struct.pack('>BBBB', 2, id_, 2, int(round(power))))
        time.sleep(.05)
else:
    last_revs = None
    last_t = None
    while True:
        t = time.time()
        p.write_packet(struct.pack('>BBB', 2, id_, 1))
        for pkt in p.read_packet():
            #print pkt
            if pkt[0] == 3 and pkt[1] == id_:
                revs = pkt[2]
                break
        if last_revs is not None:
            delta_revs = (revs - last_revs) % 256
            delta_t = t - last_t
            print delta_revs, delta_revs/delta_t
        last_revs = revs
        last_t = t
        time.sleep(.3)
