from __future__ import division

import sys

import intelhex

import bootloader


ih = intelhex.IntelHex(sys.argv[1])

assert 0x1c00 not in ih.addresses()

ih[0x1c00] = int(sys.argv[2])

ih.write_hex_file(sys.argv[3])
