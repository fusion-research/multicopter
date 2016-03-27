from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/serial/by-id/pci-FTDI_FT232R_USB_UART_AH034I96-if00-port0', 115200, timeout=.1)

p = protocol.Protocol(s)

for i in xrange(200, 500+1, 10):
    print i
    for id_ in [3]: #xrange(6):
        dev = protocol.Device(p, 2, id_)
        dev.write_packet(struct.pack('<BH', 2, i))
    time.sleep(.01)

time.sleep(3)

while True:
    while dev.read_packet() is not None: pass
    
    #dev.write_packet(struct.pack('<BH', 2, 0))
    dev.write_packet(struct.pack('<B', 3)) # start capture
    time.sleep(.1)
    
    res = []
    for i in xrange(511):
        dev.write_packet(struct.pack('>BH', 4, i)) # read
        #res.append(ord(dev.read_packet()))
        res.append((ord(dev.read_packet()) + 0x80) % 0x100 - 0x80)
    
    print res
