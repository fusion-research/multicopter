from __future__ import division

import time
import random
import binascii
import struct

import serial

import protocol

class Bootloader(object):
    def __init__(self, s, id_):
        self._id = id_
        self._s = s
        self._s.read(self._s.inWaiting())

    def _write_packet(self, payload):
        assert len(payload) == 21
        d = 'd830037d'.decode('hex')
        d += payload
        d += struct.pack('>I', binascii.crc32(d) & 0xffffffff)
        d = '\xff' + d
        #print 'SEND', d.encode('hex')
        self._s.write(d)

    def _read_packet(self, length):
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
            d = self._s.read(max(1, self._s.inWaiting()))
            #print 'RECV', d.encode('hex')
            assert d
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

    def read_page(self, page_number):
        self._write_packet(struct.pack('>BB18xB', 0, page_number, self._id))
        return self._read_packet(512)
    def write(self, addr, data):
        assert len(data) <= 16
        p = struct.pack('>BBH16sB', 1, len(data), addr, data, self._id)
        self._write_packet(p)
        if self._read_packet(len(p)) != p: raise ValueError()
    def erase_page(self, page_number):
        #assert 2 <= page_number < 14
        p = struct.pack('>BB18xB', 2, page_number, self._id)
        self._write_packet(p)
        if self._read_packet(len(p)) != p: raise ValueError()
    def run_program(self):
        p = struct.pack('>B19xB', 3, self._id)
        self._write_packet(p)
        if self._read_packet(len(p)) != p: raise ValueError()

class Bootloader2(object):
    def __init__(self, s, id_):
        self._p = protocol.Protocol(s)
        self._id = id_
    
    def _write_packet(self, payload):
        self._p.write_packet(struct.pack('>BB', 0, self._id) + payload)
    
    def _read_packet(self):
        for pkt in self._p.read_packet():
            #print pkt
            if pkt[0] == 1 and pkt[1] == self._id:
                return ''.join(map(chr, pkt[2:]))

    def read_page(self, page_number):
        self._write_packet(struct.pack('>BB', 0, page_number))
        return self._read_packet()
    def write(self, addr, data):
        assert len(data) <= 16
        p = struct.pack('>BBH16s', 1, len(data), addr, data)
        self._write_packet(p)
        if self._read_packet() != p: raise ValueError()
    def erase_page(self, page_number):
        #assert 2 <= page_number < 14
        p = struct.pack('>BB', 2, page_number)
        self._write_packet(p)
        if self._read_packet() != p: raise ValueError()
    def run_program(self):
        p = struct.pack('>B', 3)
        self._write_packet(p)
        if self._read_packet() != p: raise ValueError()
