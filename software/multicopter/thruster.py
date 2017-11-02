from __future__ import division
import numpy as np; npl = np.linalg


class Thruster(object):
    """
    Defines a single thruster (motor + propeller) of a multicopter.

    position:             3-vector to the center of this thruster in body-coordinates
    thrust_from_effort:   function that takes an effort value and returns a thrust value
    reaction_from_effort: function that takes an effort value and returns the propeller's axial reaction torque
    max_effort:           maximum effort commandable
    min_effort:           minimum effort commandable (can be negative)
    direction:            3-vector along the positive-effort direction of this thruster in body-coordinates

    Notes:
        - "Effort" usually corresponds to RPM, but could mean voltage, PWM, etc...
        - Origin of body-coordinates must be this multicopter's center of mass
        - Typically, reaction_from_effort is just some fraction of propeller_diameter*thrust_from_effort(effort)*handedness
        - Assume negligible actuation slew
        - Assume negligible chassis flexing
        - Assume negligible motor magnetic leakage

    """
    def __init__(self, position, thrust_from_effort, reaction_from_effort, max_effort, min_effort=0, direction=[0, 0, 1]):
        self.position = np.array(position, dtype=np.float64)
        self.thrust_from_effort = thrust_from_effort
        self.reaction_from_effort = reaction_from_effort
        self.max_effort = float(max_effort)
        self.min_effort = float(min_effort)
        self.direction = np.array(direction, dtype=np.float64) / npl.norm(direction)
