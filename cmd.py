from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=.01)
id_ = int(sys.argv[1])

p = protocol.Protocol(s)


last_revs = None
last_time = None
def stat():
    global last_revs, last_time
    p.write_packet(struct.pack('>BBB', 2, id_, 1))
    while True:
        pkt = p.read_packet()
        if pkt[0] == 3 and pkt[1] == id_ and len(pkt) == 8:
            revs, time_ = struct.unpack('<HI', ''.join(map(chr, pkt[2:])))
            if last_revs is not None:
                drevs = (revs - last_revs) % 2**16 / 6
                dt = (time_ - last_time) % 2**32 / 24.5e6
                print 'STAT', drevs/dt
            last_revs = revs
            last_time = time_
            break

for i in xrange(200, 500):
    print i
    p.write_packet(struct.pack('<BBBH', 2, id_, 2, i))
    stat()
    time.sleep(.01)

while True:
    stat()
