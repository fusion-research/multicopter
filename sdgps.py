from __future__ import division

import struct

from twisted.internet import protocol

import raw

import datachunker


class SDGPSProtocol(protocol.Protocol):
    def __init__(self):
        self.dataReceived = datachunker.DataChunker(self.dataReceiver())
        self.last_imu = None
    
    def dataReceiver(self):
        while True:
            id_, timestamp, size = struct.unpack('<BqI', (yield 13))
            data = yield size
            packet = raw.DataType.unpack(id_, timestamp, data)
            
            if isinstance(packet, raw.IMUPacket):
                self.last_imu = packet
