import struct
import time

from twisted.internet import serialport, protocol, defer

import datachunker, deferral, variable
    
def _escape(data):
    res = []
    for b in data:
        if b in '7e7d1113'.decode('hex'):
            res.append(chr(0x7d))
            res.append(chr(ord(b) & 0x20))
        else:
            res.append(b)
    return ''.join(res)

def _calculate_checksum(data):
    return 0xFF - sum(map(ord, data)) % 256

class Protocol(protocol.Protocol):
    def connectionLost(self, reason):
        print 'XBee connection lost:', reason

class ReadeyThing(object):
    def __init__(self):
        self._state = None
        self.dataReceived = datachunker.DataChunker(self.dataReceiver())
    
    def dataReceiver(self):
        assert self._state is None
        c = yield 1
        while True:
            assert self._state is not None
            length, df = self._state
            self._state = None
            data = c + (yield length - len(c))
            c = ''
            assert self._state is None
            df.callback(data)
            assert self._state is not None
    
    def read(self, length):
        assert length >= 1
        assert self._state is None
        df = defer.Deferred()
        self._state = length, df
        return df

class XBee(object):
    @defer.inlineCallbacks
    def __new__(cls, reactor, port, initial_baud_rate=9600):
        self = object.__new__(XBee)
        
        buf = []
        self._protocol = Protocol()
        self._protocol.dataReceived = buf.extend
        
        self._port = serialport.SerialPort(self._protocol, port, reactor, initial_baud_rate)
        self._port.flushInput()
        yield deferral.sleep(1.1)
        print repr(''.join(buf)); buf = []
        self._port.write('+++')
        yield deferral.sleep(1.1)
        print repr(''.join(buf)); buf = []
        self._port.write('ATAP2\r')
        yield deferral.sleep(.1)
        print repr(''.join(buf)); buf = []
        self._port.write('ATRR0\r')
        yield deferral.sleep(.1)
        print repr(''.join(buf)); buf = []
        self._port.write('ATMT0\r')
        yield deferral.sleep(.1)
        print repr(''.join(buf)); buf = []
        self._port.write('ATCN\r')
        yield deferral.sleep(.1)
        print repr(''.join(buf)); buf = []
        
        self._reader = ReadeyThing()
        def x(data):
            print len(data), data.encode('hex')
            self._reader.dataReceived(data)
        self._protocol.dataReceived = x
        
        self.packet_received = variable.Event()
        
        self._data_reader()
        
        defer.returnValue(self)
    
    @defer.inlineCallbacks
    def _data_reader(self):
        while True:
            x = yield self._reader.read(1)
            start = time.time()
            if x != '\x7e':
                print 'garbage', x.encode('hex')
                continue
            
            length, = struct.unpack('>H', (yield self._read_unescaped(2)))
            if length == 0:
                print 'length == 0'
                continue
            if length > 270:
                print 'length too long', length
                continue
            
            data = yield self._read_unescaped(length)
            
            checksum_should = _calculate_checksum(data)
            checksum, = struct.unpack('>B', (yield self._read_unescaped(1)))
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
                print 'decode took', time.time() - start
                self.packet_received.happened(dict(source_address=source_address, rssi=rssi, options=options, data=cmdData[4:]))
    
    @defer.inlineCallbacks
    def _read_unescaped(self, length):
        res = ''
        while len(res) < length:
            c = yield self._reader.read(1)
            if c == '\x7d':
                x = yield self._reader.read(1)
                res += chr(ord(x) ^ 0x20)
            else:
                res += c
        defer.returnValue(res)
    
    def _send_packet(self, frame_data):
        res = []
        res.append(chr(0x7e))
        res.extend(_escape(struct.pack('>H', len(frame_data)) + frame_data + chr(_calculate_checksum(frame_data))))
        self._port.write(''.join(res))
    
    def transmit(self, data):
        start = time.time()
        self._send_packet('0100ffff00'.decode('hex') + data)
        print 'transmit took', time.time() - start
