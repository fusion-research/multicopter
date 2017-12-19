"""
A quadcopter model and a hexcopter model are configured here for easy access and reuse by other scripts.
If this script is run as main, then a C++ header file for the hexcopter model will be generated into the ./autopilot/ folder.

"""
from __future__ import division
import numpy as np; npl = np.linalg
from collections import OrderedDict

from model import Model
from thruster import Thruster

# Chassis characteristics
mass = 0.5  # kg
inertia = np.diag((5e-3, 5e-3, 9e-3))  # kg*m^2
drag_lin = np.array([5e-1, 5e-1, 5e-1])  # N/(m/s)
drag_ang = np.array([2e-2, 2e-2, 2e-2])  # N/(rad/s)
r = 0.28  # m, radial distance from center of mass to motor mounts

# Motor characteristics
motor_kv = 920  # rpm / V
bus_voltage = 0.9 * 14  # V, 90% of full charge
max_rpm = motor_kv * bus_voltage

# Propeller and aero characteristics
prop_dia = 0.2  # m
effectiveness = 1e-7  # N/(rpm^2)
thrust_from_effort = lambda rpm: effectiveness * np.abs(rpm) * rpm  # N
effort_from_thrust = lambda thrust: np.sign(thrust) * np.sqrt(np.abs(thrust) / effectiveness)  # rpm
reaction_coeff = 0.25 * prop_dia  # (N*m)/N

# Quadcopter thrusters (sign of reaction_coeff determines handedness)
quad_thrusters = OrderedDict([("front", Thruster(( r,  0, 0.02), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm)),
                              ("left",  Thruster(( 0,  r, 0.02), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm)),
                              ("back",  Thruster((-r,  0, 0.02), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm)),
                              ("right", Thruster(( 0, -r, 0.02), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm))])

# Multicopter thrusters (sign of reaction_coeff determines handedness)
R = np.array([[np.cos(2*np.pi/12), -np.sin(2*np.pi/12), 0],[np.sin(2*np.pi/12), np.cos(2*np.pi/12), 0], [0, 0, 1]])
hex_thrusters = OrderedDict([("fl", Thruster(npl.matrix_power(R, 1).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm)),
                             ("ml", Thruster(npl.matrix_power(R, 3).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm)),
                             ("bl", Thruster(npl.matrix_power(R, 5).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm)),
                             ("br", Thruster(npl.matrix_power(R, 7).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm)),
                             ("mr", Thruster(npl.matrix_power(R, 9).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm)),
                             ("fr", Thruster(npl.matrix_power(R, 11).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm))])

# Model objects
quadboi = Model(quad_thrusters, mass, inertia, drag_lin, drag_ang)
hexboi = Model(hex_thrusters, mass, inertia, drag_lin, drag_ang)

# One more with no gravity for fun (needs reversible thrusters)
hex_thrusters_zg = OrderedDict([("fl", Thruster(npl.matrix_power(R, 1).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm, -max_rpm)),
                                ("ml", Thruster(npl.matrix_power(R, 3).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm, -max_rpm)),
                                ("bl", Thruster(npl.matrix_power(R, 5).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm, -max_rpm)),
                                ("br", Thruster(npl.matrix_power(R, 7).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm, -max_rpm)),
                                ("mr", Thruster(npl.matrix_power(R, 9).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, reaction_coeff, max_rpm, -max_rpm)),
                                ("fr", Thruster(npl.matrix_power(R, 11).dot([r, 0, 0.02]), thrust_from_effort, effort_from_thrust, -reaction_coeff, max_rpm, -max_rpm))])
hexboi_zg = Model(hex_thrusters_zg, mass, inertia, drag_lin, drag_ang, gravity=[0, 0, 0])

# One more with randomly distributed thrusters for fun
rando_thrusters = OrderedDict()
for i in xrange(10):
    rando_thrusters[str(i)] = Thruster(2*(np.random.sample(3)-0.5), thrust_from_effort, effort_from_thrust,
                                       reaction_coeff, max_rpm, -max_rpm, np.random.sample(3)-0.5)
randoboi = Model(rando_thrusters, mass, inertia, drag_lin, drag_ang)

# Generate C++ header for use by autopilot backend
if __name__ == "__main__":
    hexboi.make_header(directory="autopilot")  # autopilot expects a hexcopter
