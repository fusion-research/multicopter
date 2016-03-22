from __future__ import division

import struct
import binascii


ESCAPE        = 0x34
ESCAPE_START  = 0x01
ESCAPE_END    = 0x02
ESCAPE_ESCAPE = 0x03

class Protocol(object):
    def __init__(self, s):
        self._s = s
        self._reader = self._reader_generator()
    
    def write_packet(self, data):
        #print 'SEND', data.encode('hex')
        data = data + struct.pack('<I', binascii.crc32(data) & 0xffffffff)
        res = '\xff' + chr(ESCAPE) + chr(ESCAPE_START) + data.replace(chr(ESCAPE), chr(ESCAPE) + chr(ESCAPE_ESCAPE)) + chr(ESCAPE) + chr(ESCAPE_END)
        self._s.write(res)
    
    def read_packet(self):
        return self._reader.next()
    
    def _reader_generator(self):
        in_message = False
        in_escape = False
        buf = []
        while True:
            d = self._s.read(max(1, self._s.inWaiting()))
            if not d:
                yield None
            for byte in d:
                byte = ord(byte)
                if byte == ESCAPE:
                    if in_escape:
                        print 'err0'
                        in_message = False # shouldn't happen, reset
                    in_escape = True
                elif in_escape:
                    in_escape = False
                    if byte == ESCAPE_START:
                        buf = []
                        in_message = True
                    elif byte == ESCAPE_ESCAPE and in_message:
                        buf.append(ESCAPE)
                    elif byte == ESCAPE_END and in_message:
                        if binascii.crc32(''.join(map(chr, buf))) & 0xffffffff == 0x2144df1c:
                            #print 'RECV', ''.join(map(chr, buf)).encode('hex')
                            yield buf[:-4]
                        else:
                            print 'invalid crc', ''.join(map(chr, buf)).encode('hex')
                        in_message = False
                    else:
                        print 'err1'
                        in_message = False # shouldn't happen, reset
                else:
                    if in_message:
                        buf.append(byte)
                    else:
                        #print 'err2', byte
                        pass # shouldn't happen, ignore
