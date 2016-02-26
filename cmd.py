from __future__ import division

import struct
import time
import random

import serial


s = serial.Serial('/dev/ttyUSB0', 115200)

def read_packet(length):
    class GotResult(Exception):
        def __init__(self, res):
            self.res = res
    def _reader():
        if (yield) != '\xa4': return
        if (yield) != '\x76': return
        if (yield) != '\x6a': return
        if (yield) != '\x7f': return
        d = []
        for i in xrange(length): d.append((yield))
        raise GotResult(''.join(d))
    readers = []
    while True:
        d = s.read(max(1, s.inWaiting()))
        for c in d:
            readers.append(_reader())
            readers[-1].send(None)
            new_readers = []
            for r in readers:
                try:
                    r.send(c)
                except StopIteration:
                    pass
                except GotResult as e:
                    return e.res
                else:
                    new_readers.append(r)
            readers = new_readers

for i in xrange(150, 170):
    print i
    s.write('3719153c'.decode('hex') + chr(i))
    time.sleep(.03)

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
