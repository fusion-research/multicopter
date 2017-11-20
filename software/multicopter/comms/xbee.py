import struct
import time

from twisted.internet import serialport, protocol, defer

import datachunker, deferral, variable
    
def _calculate_checksum(data):
    return 0xFF - sum(map(ord, data)) % 256

class Protocol(protocol.Protocol):
    def connectionLost(self, reason):
        print 'XBee connection lost:', reason

class XBee(object):
    @defer.inlineCallbacks
    def __new__(cls, reactor, port, initial_baud_rate=9600):
        self = object.__new__(XBee)
        
        buf = []
        self._protocol = Protocol()
        self._protocol.dataReceived = lambda data: buf.extend(data)
        
        cmds = [
            'ATID9FF',
            'ATHP6',
            'ATKY0',
            'ATRR0',
            'ATMT0',
            'ATAP1',
            'ATMYFFFF',
            'ATDTFFFF',
            #'ATMK0', # sniffing
            # RB/PK?
            'ATCN',
        ]
        
        self._port = serialport.SerialPort(self._protocol, port, reactor, initial_baud_rate)
        self._port.flushInput()
        yield deferral.sleep(1.1)
        buff_str = repr(''.join(buf)); buf = []
        if buff_str == "''": print "buffer: empty"
        else: print "buffer:", buff_str
        self._port.write('+++')
        yield deferral.sleep(1.1)
        for cmd in cmds:
            print repr(''.join(buf)); buf = []
            self._port.write(cmd + '\r')
            yield deferral.sleep(.1)
        
        self.packet_received = variable.Event()
        self._protocol.dataReceived = datachunker.DataChunker(self.dataReceiver())
        
        defer.returnValue(self)
    
    def dataReceiver(self):
        while True:
            x = yield 1
            if x != '\x7e':
                print 'garbage', x.encode('hex')
                continue
            
            length, = struct.unpack('>H', (yield 2))
            if length == 0:
                print 'length == 0'
                continue
            if length > 270:
                print 'length too long', length
                continue
            
            data = yield length
            
            checksum_should = _calculate_checksum(data)
            checksum, = struct.unpack('>B', (yield 1))
            if checksum != checksum_should:
                print 'invalid checksum!'
                continue
            
            cmdID = ord(data[0])
            cmdData = data[1:]
            
            if cmdID != 0x81:
                print 'unknown xbee packet:', (cmdID, cmdData)
                continue
            else:
                if len(cmdData) < 4:
                    print 'short xbee packet:', (cmdID, cmdData)
                    continue
                source_address, rssi, options = struct.unpack('>HBB', cmdData[:4])
                self.packet_received.happened(dict(source_address=source_address, rssi=rssi, options=options, data=cmdData[4:]))
    
    def _send_packet(self, frame_data):
        self._port.write(struct.pack('>BH', 0x7e, len(frame_data)) + frame_data + chr(_calculate_checksum(frame_data)))
    
    def transmit(self, data):
        self._send_packet('0100ffff00'.decode('hex') + data)
