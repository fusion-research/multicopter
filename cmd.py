from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/serial/by-id/pci-FTDI_FT232R_USB_UART_AH034I96-if00-port0', 115200, timeout=.01)
id_ = int(sys.argv[1])

p = protocol.Protocol(s)
dev = protocol.Device(p, 2, id_)

P = 21
I = 35
D = 0

SETPOINT = 100

error_integral = 0

last_revs = None
last_time = None
def stat():
    global last_revs, last_time, error_integral
    p.write_packet(struct.pack('>B',, 1))
    while True:
        pkt = dev.read_packet()
        if len(pkt) == 6:
            revs, time_ = struct.unpack('<HI', pkt)
            if last_revs is not None:
                drevs = (revs - last_revs) % 2**16 / 6 / 7
                dt = (time_ - last_time) % 2**32 / 24.5e6
                speed = drevs/dt
                error = SETPOINT - speed
                error_integral += I * (error * dt)
                output = P * (SETPOINT - speed) + error_integral
                output = int(round(output))
                if error_integral > 2000: error_integral = 2000
                if output > 2000: output = 2000
                if output < 200: output = 200
                print 'STAT', drevs*6*7, drevs/dt, error_integral, output
                dev.write_packet(struct.pack('<BH', 2, output))
            last_revs = revs
            last_time = time_
            break

'''for i in xrange(200, 500):
    print i
    dev.write_packet(struct.pack('<BH', 2, i))
    stat()
    time.sleep(.01)'''

while dev.read_packet() is not None: pass

for i in xrange(200, 300):
    print i
    dev.write_packet(struct.pack('<BH', 2, i))
    time.sleep(.01)

while True:
    stat()
    time.sleep(.01)
