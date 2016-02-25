from __future__ import division

import time
import random
import binascii
import struct

import serial


s = serial.Serial('/dev/ttyUSB0', 115200)

def write_packet(payload):
    assert len(payload) <= 20
    payload = payload + chr(0) * (20 - len(payload))
    d = 'd830037d'.decode('hex')
    d += payload
    d += struct.pack('>I', binascii.crc32(d) & 0xffffffff)
    s.write(d)
    print 'send', d.encode('hex')

def read_packet():
    def _reader():
        if (yield) != '\xe4': return
        if (yield) != '\x8c': return
        if (yield) != '\xf1': return
        if (yield) != '\xcb': return
        d = ['\xe4\x8c\xf1\xcb']
        for i in xrange(512 + 4): d.append((yield))
        d = ''.join(d)
        if d[-4:] != struct.pack('>I', binascii.crc32(d[:-4]) & 0xffffffff):
            print 'data', d.encode('hex'), len(d)
            print 'bad crc', d[-4:].encode('hex'), struct.pack('>I', binascii.crc32(d[:-4]) & 0xffffffff).encode('hex')
            return
        raise GotResult(d[4:-4])
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
                else:
                    new_readers.append(r)
            readers = new_readers

s.read(s.inWaiting())
write_packet('\x00\x02') # read page 2
print read_packet().encode('hex')
