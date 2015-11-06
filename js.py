from __future__ import division

import evdev

dev = evdev.device.InputDevice('/dev/input/by-id/usb-Logitech_Inc._WingMan_Extreme_Digital_3D-event-joystick')

abs_states = {}
key_states = {}
for type_, value in dev.capabilities().iteritems():
    if type_ == evdev.ecodes.EV_KEY:
        for code in value:
            key_states[code] = 0
    elif type_ == evdev.ecodes.EV_ABS:
        for code, absinfo in value:
            abs_states[code] = absinfo
for code, value in dev.active_keys():
    key_states[code] = value

for event in dev.read_loop():
    if event.type == evdev.ecodes.EV_SYN and event.code == evdev.ecodes.SYN_REPORT and event.value == 0:
        continue
    elif event.type == evdev.ecodes.EV_ABS:
        abs_states[event.code] = abs_states[event.code]._replace(value=event.value)
    elif event.type == evdev.ecodes.EV_MSC and event.code == evdev.ecodes.MSC_SCAN:
        continue # only happens for buttons, but they get EV_KEY, which is more useful
    elif event.type == evdev.ecodes.EV_KEY:
        key_states[event.code] = event.value
    else:
        print 'unknown event:', map(hex, (event.type, event.code, event.value))
    
    print abs_states, key_states
