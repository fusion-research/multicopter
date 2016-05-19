from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/serial/by-id/pci-FTDI_FT232R_USB_UART_AH034I96-if00-port0', 115200, timeout=.1)

p = protocol.Protocol(s)

id_ = 2
dev = protocol.Device(p, 2, id_)

def print_status():
    dev.write_packet(struct.pack('<B', 1))
    x = dev.read_packet()
    if x is None: return
    revs, controller_output, controller_speed_measured, t, controller_integral = struct.unpack('<HhHIi', x)
    print revs, controller_output, controller_speed_measured, t, controller_integral

for i in xrange(100, 250):
    print i
    print_status()
    dev.write_packet(struct.pack('<BH', 2, i))
    time.sleep(.04)

print 'starting controller'
dev.write_packet(struct.pack('<BH', 5, 5000))
t0 = time.time()

if 0:
    while True:
        print_status()
else:
    while True:
        import math
        time.sleep(.01)
        dev.write_packet(struct.pack('<BH', 5, 5000 + 1000 * math.sin(6.28*(time.time()-t0))))
        print_status()
