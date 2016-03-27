from __future__ import division

import struct
import sys

import serial
import intelhex

import bootloader
from bootloader import PAGE_SIZE
import protocol


id_ = int(sys.argv[1])
id2_ = int(sys.argv[2])

prot = protocol.Protocol(serial.Serial('/dev/serial/by-id/pci-FTDI_FT232R_USB_UART_AH034I96-if00-port0', 115200, timeout=1))
b = bootloader.Bootloader2(prot, id_)
b2 = bootloader.Bootloader2(prot, id2_)

d = {}
d2 = {}

for p in xrange(15):
    print 'reading page', p
    d.update(dict(zip(xrange(PAGE_SIZE*p, PAGE_SIZE*(p+1)), map(ord, b.read_page(p)))))
    d2.update(dict(zip(xrange(PAGE_SIZE*p, PAGE_SIZE*(p+1)), map(ord, b2.read_page(p)))))

for i in d:
    if d[i] != d2[i]:
        print hex(i), hex(d[i]), hex(d2[i])
