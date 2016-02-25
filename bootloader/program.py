from __future__ import division

import time
import random
import binascii
import struct

import serial


s = serial.Serial('/dev/ttyUSB0', 115200)

def write_packet(payload):
    assert len(payload) == 20
    d = 'd830037d'.decode('hex')
    d += payload
    d += struct.pack('>I', binascii.crc32(d) & 0xffffffff)
    s.write(d)
    #print 'send', d.encode('hex')

def read_packet(length):
    class GotResult(Exception):
        def __init__(self, res):
            self.res = res
    def _reader():
        if (yield) != '\xe4': return
        if (yield) != '\x8c': return
        if (yield) != '\xf1': return
        if (yield) != '\xcb': return
        d = ['\xe4\x8c\xf1\xcb']
        for i in xrange(length + 4): d.append((yield))
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
                except GotResult as e:
                    return e.res
                else:
                    new_readers.append(r)
            readers = new_readers

def read_page(page_number):
    write_packet(struct.pack('>BB18x', 0, page_number))
    return read_packet(512)
def write(addr, data):
    assert len(data) <= 16
    p = struct.pack('>BBH16s', 1, len(data), addr, data)
    write_packet(p)
    if read_packet(len(p)) != p: raise ValueError()
def erase_page(page_number):
    assert 2 <= page_number <= 14
    p = struct.pack('>BB18x', 2, page_number)
    write_packet(p)
    if read_packet(len(p)) != p: raise ValueError()
def run_program():
    p = struct.pack('>B19x', 3)
    write_packet(p)
    if read_packet(len(p)) != p: raise ValueError()

s.read(s.inWaiting())

d = ''
for p in xrange(15): d += read_page(p)
print d.encode('hex')
print 'erasing'
erase_page(8)
print 'writing'
write(0x1000, ''.join(map(chr, xrange(16)))[::-1])
print 'done'
d = ''
for p in xrange(15): d += read_page(p)
print d.encode('hex')
print d[0x1000-3:0x1000+20].encode('hex')
