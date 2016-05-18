from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/serial/by-id/pci-FTDI_FT232R_USB_UART_AH034I96-if00-port0', 115200, timeout=.1)

p = protocol.Protocol(s)

id_ = 3
dev = protocol.Device(p, 2, id_)

for i in xrange(100, 250):
    print i
    dev.write_packet(struct.pack('<BH', 2, i))
    time.sleep(.04)

dev.write_packet(struct.pack('<BH', 5, 5000))

if 0:
    while True:
        dev.write_packet(struct.pack('<B', 1))
        x = dev.read_packet()
        if x is None: continue
        revs, controller_on_time, controller_speed_measured, t, controller_integral = struct.unpack('<HHHIi', x)
        print revs, controller_on_time, controller_speed_measured, t, controller_integral
else:
    while True:
        import math
        time.sleep(.01)
        dev.write_packet(struct.pack('<BH', 5, 1750 + 500 * math.sin(6.28*time.time())))
        dev.write_packet(struct.pack('<B', 1))
        x = dev.read_packet()
        if x is None: continue
        revs, controller_on_time, controller_speed_measured, t, controller_integral = struct.unpack('<HHHIi', x)
        print revs, controller_on_time, controller_speed_measured, t, controller_integral
