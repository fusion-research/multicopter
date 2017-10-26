from __future__ import division

from twisted.internet import abstract
import evdev

import variable


class JS(abstract.FileDescriptor):
    def __init__(self, reactor):
        abstract.FileDescriptor.__init__(self, reactor)
        
        self._dev = evdev.device.InputDevice('/dev/input/by-id/usb-Logitech_Inc._WingMan_Extreme_Digital_3D-event-joystick')

        abs_states = {}
        key_states = {}
        for type_, value in self._dev.capabilities().iteritems():
            if type_ == evdev.ecodes.EV_KEY:
                for code in value:
                    key_states[code] = 0
            elif type_ == evdev.ecodes.EV_ABS:
                for code, absinfo in value:
                    abs_states[code] = absinfo
        for code in self._dev.active_keys():
            key_states[code] = 1
        
        self.state = variable.Variable((abs_states, key_states))
        
        reactor.addReader(self)
    
    def fileno(self):
        return self._dev.fileno()
    def doRead(self):
        event = self._dev.read_one()
        
        if event.type == evdev.ecodes.EV_SYN and event.code == evdev.ecodes.SYN_REPORT and event.value == 0:
            return
        elif event.type == evdev.ecodes.EV_ABS:
            self.state.set((dict(self.state.value[0], **{event.code: self.state.value[0][event.code]._replace(value=event.value)}), self.state.value[1]))
        elif event.type == evdev.ecodes.EV_MSC and event.code == evdev.ecodes.MSC_SCAN:
            return # only happens for buttons, but they get EV_KEY, which is more useful
        elif event.type == evdev.ecodes.EV_KEY:
            self.state.set((self.state.value[0], dict(self.state.value[1], **{event.code: event.value})))
        else:
            print 'unknown event:', map(hex, (event.type, event.code, event.value))
