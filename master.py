
import os
import struct

fd = os.open('/dev/input/by-id/usb-Logitech_Inc._WingMan_Extreme_Digital_3D-event-joystick', os.O_RDONLY)

abs_codes = {0: 'x', 1: 'y', 5: 'Z', 6: 'throttle', 16: 'hat_x', 17: 'hat_y'}
key_codes = {0x120: 'trigger', 0x121: 'left_up', 0x122: 'left_down', 0x123: 'right_up', 0x124: 'right_down', 0x125: 'O6', 0x126: 'O7'}

abs_states = {}
key_states = {}

while True:
    FORMAT = 'llHHi'
    EVENT_SIZE = struct.calcsize(FORMAT)
    tv_sec, tv_usec, type_, code, value = struct.unpack(FORMAT, os.read(fd, EVENT_SIZE))
    if type_ == 0 and code == 0 and value == 0:
        continue
    elif type_ == 3 and code in abs_codes:
        abs_states[abs_codes[code]] = value
    elif type_ == 4 and code == 4:
        continue # EV_MSC - only happens for buttons, but they get EV_KEY, which is more useful
    elif type_ == 1 and code in key_codes:
        key_states[key_codes[code]] = value
    else:
        print 'unknown event:', map(hex, (type_, code, value))
        assert False
    print abs_states, key_states
