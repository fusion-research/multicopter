from __future__ import division

import struct

import protocol

PAGE_SIZE = 512

class Bootloader2(object):
    def __init__(self, s, id_, upgrader_mode=False):
        self._dev = protocol.Device(s, 0 if not upgrader_mode else 4, id_)
    
    def read_page(self, page_number):
        self._dev.write_packet(struct.pack('>BB', 0, page_number))
        return self._dev.read_packet()
    def write(self, addr, data):
        assert len(data) <= 16
        p = struct.pack('>BBH16s', 1, len(data), addr, data)
        self._dev.write_packet(p)
        if self._dev.read_packet() != p: raise ValueError()
    def erase_page(self, page_number):
        #assert 2 <= page_number < 14
        p = struct.pack('>BB', 2, page_number)
        self._dev.write_packet(p)
        if self._dev.read_packet() != p: raise ValueError()
    def run_program(self):
        p = struct.pack('>B', 3)
        self._dev.write_packet(p)
        if self._dev.read_packet() != p: raise ValueError()
