import struct
import time

import serial
    
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

class XBee(object):
    def __init__(self, port, initial_baud_rate=9600):
        self.s = serial.Serial(port, initial_baud_rate)
        self.s.flushInput()
        
        time.sleep(1.1)
        self.s.write('+++')
        time.sleep(1.1)
        print repr(self.s.read(self.s.inWaiting()))
        self.s.write('ATAP2\r')
        time.sleep(.1)
        print repr(self.s.read(self.s.inWaiting()))
        self.s.write('ATCN\r')
        time.sleep(.1)
        print repr(self.s.read(self.s.inWaiting()))
    
    def _read_unescaped(self, length):
        res = ''
        while len(res) < length:
            c = self.s.read(1)
            if c == '\x7d':
                x = self.s.read(1)
                res += chr(ord(x) ^ 0x20)
            else:
                res += c
        return res
    
    def _send_packet(self, frame_data):
        res = []
        res.append(chr(0x7e))
        res.extend(_escape(struct.pack('>H', len(frame_data)) + frame_data + chr(_calculate_checksum(frame_data))))
        self.s.write(''.join(res))
    
    def transmit(self, data):
        self._send_packet('0100ffff00'.decode('hex') + data)
    
    def read(self):
        while True:
            x = self.s.read(1)
            if x != '\x7e':
                print 'garbage', x.encode('hex')
                continue
            
            length, = struct.unpack('>H', self._read_unescaped(2))
            if length == 0:
                print 'length == 0'
                continue
            if length > 270:
                print 'length too long', length
                continue
            
            data = self._read_unescaped(length)
            
            checksum_should = _calculate_checksum(data)
            checksum, = struct.unpack('>B', self._read_unescaped(1))
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
                return dict(source_address=source_address, rssi=rssi, options=options, data=cmdData[4:])
