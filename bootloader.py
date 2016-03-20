from __future__ import division

import struct

import protocol

class Bootloader2(object):
    def __init__(self, s, id_):
        self._p = protocol.Protocol(s)
        self._id = id_
    
    def _write_packet(self, payload):
        self._p.write_packet(struct.pack('>BB', 0, self._id) + payload)
    
    def _read_packet(self):
        while True:
            pkt = self._p.read_packet()
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
