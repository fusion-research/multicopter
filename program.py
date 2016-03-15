from __future__ import division

import struct
import sys

import serial
import intelhex

import bootloader
import protocol


ih = intelhex.IntelHex(sys.argv[1])
id_ = int(sys.argv[2])

b = bootloader.Bootloader(serial.Serial('/dev/ttyUSB0', 115200, timeout=1), id_)
p = protocol.Protocol(serial.Serial('/dev/ttyUSB0', 115200, timeout=1))

print 'resetting'
for i in xrange(10):
    p.write_packet(struct.pack('>BBB', 2, id_, 0)) # tell user code to reset into bootloader

PAGE_SIZE = 512

d = {a: ih[a] for a in ih.addresses()}
pages = {a//PAGE_SIZE for a in d}

device_data = {}
for p in pages:
    print 'reading page', p
    device_data.update(dict(zip(xrange(PAGE_SIZE*p, PAGE_SIZE*(p+1)), map(ord, b.read_page(p)))))

dirty_pages = {a//PAGE_SIZE for a in d if device_data[a] != d[a]}
print 'dirty pages: {%s}' % (', '.join(map(str, dirty_pages)),)

if dirty_pages:
    for p in dirty_pages:
        if all(device_data[a] == 0xff for a in xrange(PAGE_SIZE*p, PAGE_SIZE*(p+1))):
            print 'page', p, 'already erased'
        else:
            print 'erasing page', p
            b.erase_page(p)
            for a in xrange(PAGE_SIZE*p, PAGE_SIZE*(p+1)): device_data[a] = 0xff

    dirty_addresses = {a for a in d if device_data[a] != d[a]}

    reversed_sorted_dirty_addresses = sorted(dirty_addresses, reverse=True)
    while reversed_sorted_dirty_addresses:
        start_addr = reversed_sorted_dirty_addresses.pop()
        end_addr_inclusive = start_addr
        while reversed_sorted_dirty_addresses and reversed_sorted_dirty_addresses[-1] < start_addr + 16:
            end_addr_inclusive = reversed_sorted_dirty_addresses.pop()
        
        data = ''.join(chr(d.get(a, 0xff)) for a in xrange(start_addr, end_addr_inclusive+1))
        
        print 'writing', len(data), 'bytes to', hex(start_addr)
        
        b.write(start_addr, data)

    print 'verifying...'

    device_data = {}
    for p in pages:
        print 'reading page', p
        device_data.update(dict(zip(xrange(PAGE_SIZE*p, PAGE_SIZE*(p+1)), map(ord, b.read_page(p)))))

    dirty_pages = {a//PAGE_SIZE for a in d if device_data[a] != d[a]}
    assert not dirty_pages

print 'done!'

print 'running program'
b.run_program()
print 'program started'
