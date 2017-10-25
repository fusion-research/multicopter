from __future__ import division

import struct
import time
import sys

import serial

import protocol

s = serial.Serial('/dev/serial/by-id/pci-FTDI_FT232R_USB_UART_AH034I96-if00-port0', 115200, timeout=.01)

p = protocol.Protocol(s)
reader = p.read_packet()


p.write_packet(struct.pack('>BBB', 2, 3, 4))

for id_ in [3]*1000: #xrange(256):
    last = time.time()
    last_t = None
    first_t = None
    first = None
    p.write_packet(struct.pack('>BBB', 2, id_, 3))
    while True:
        x = reader.next()
        if x is None: break
        if x[0] != 3: continue
        if x[1] != id_: continue
        if x[2] != 3: continue
        assert len(x) == 7, x
        t = x[3] + 2**8*x[4] + 2**16*x[5] + 2**24*x[6]
        if first_t is None: first_t = t
        now = time.time()
        if first is None: first = now
        print id_, (t - first_t)/24.5e6, now-first, x
        sys.exit()
        last = now
        
        if last_t is not None:
            assert (t - last_t) % 2**32 <= 2**31, (last_t, t)
        
        last_t = t
        p.write_packet(struct.pack('>BBB', 2, id_, 3))
    print 'timeout'
