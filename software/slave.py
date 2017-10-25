from __future__ import division

import math
import time
import struct
import os
import numpy

from twisted.internet import defer, reactor

import deferral
import xbee

STOP_PWM = 0.001
MIN_PWM =  0.00116
MAX_PWM =  0.00190

MIN_FORCE = 0
MAX_FORCE = 4.6


class Motor(object):
    def __init__(self, path):
        self.path = path
        with open(path + '/run', 'wb') as f: f.write('0')
        with open(path + '/period', 'wb') as f: f.write('20000000')
        with open(path + '/duty', 'wb') as f: f.write('100000')
        self.duty_fileno = os.open(self.path + '/duty', os.O_WRONLY)
        self.set_pwm(STOP_PWM)
        with open(path + '/run', 'wb') as f: f.write('1')
    
    def set_pwm(self, value): # value is pulse width in seconds
        assert STOP_PWM*.99 <= value <= MAX_PWM*1.01, value
        os.write(self.duty_fileno, str(int(value/1e-9+0.5)) + '\n')

class Thruster(object):
    def __init__(self, motor, position, direction, torque_per_force):
        self.motor, self.position, self.direction, self.torque_per_force = motor, position, direction, torque_per_force

@defer.inlineCallbacks
def main():
    xb = yield xbee.XBee(reactor, '/dev/ttyO0', 230400)
    
    polar = lambda r, theta: r*numpy.array([math.cos(theta), math.sin(theta), 0])
    thrusters = [
        Thruster(Motor('/sys/devices/ocp.2/pwm_test_P8_13.11'), polar(.319, math.radians(30+60*0)), numpy.array([0, 0, 1]), numpy.array([0, 0, -0.1])),
        Thruster(Motor('/sys/devices/ocp.2/pwm_test_P8_19.12'), polar(.319, math.radians(30+60*1)), numpy.array([0, 0, 1]), numpy.array([0, 0, +0.1])),
        Thruster(Motor('/sys/devices/ocp.2/pwm_test_P9_14.13'), polar(.319, math.radians(30+60*2)), numpy.array([0, 0, 1]), numpy.array([0, 0, -0.1])),
        Thruster(Motor('/sys/devices/ocp.2/pwm_test_P9_16.14'), polar(.319, math.radians(30+60*3)), numpy.array([0, 0, 1]), numpy.array([0, 0, +0.1])),
        Thruster(Motor('/sys/devices/ocp.2/pwm_test_P9_29.15'), polar(.319, math.radians(30+60*4)), numpy.array([0, 0, 1]), numpy.array([0, 0, -0.1])),
        Thruster(Motor('/sys/devices/ocp.2/pwm_test_P9_31.16'), polar(.319, math.radians(30+60*5)), numpy.array([0, 0, 1]), numpy.array([0, 0, +0.1])),
    ]
    
    wrench_from_forces = numpy.array([numpy.concatenate([
        thruster.direction, # force = direction * effort
        numpy.cross(thruster.position, thruster.direction) + thruster.torque_per_force, # torque = (position X direction + torque_per_force) * effort
    ]) for thruster in thrusters]).T
    
    semiwrench_from_forces = wrench_from_forces[2:6]
    forces_from_semiwrench = numpy.linalg.pinv(semiwrench_from_forces)
    
    while True:
        try:
            packet, = yield deferral.wrap_timeout(xb.packet_received.get_deferred(), .1)
        except deferral.TimeoutError:
            stop = True
        else:
            stop, z, tx, ty, tz = struct.unpack('<B4f', packet['data'])
        
        if stop:
            for thruster in thrusters:
                thruster.motor.set_pwm(STOP_PWM)
            continue
        
        forces = map(float, forces_from_semiwrench.dot([z, tx, ty, tz]))
        for thruster, force in zip(thrusters, forces):
            force = min(max(force, MIN_FORCE), MAX_FORCE)
            pwm = math.sqrt((force - MIN_FORCE)/(MAX_FORCE - MIN_FORCE)) * (MAX_PWM-MIN_PWM) + MIN_PWM
            thruster.motor.set_pwm(pwm)
deferral.launch_main(main)
