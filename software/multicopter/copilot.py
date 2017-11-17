from __future__ import division
import numpy as np; npl = np.linalg
from allocator import Allocator
import motion


class CoPilot(object):
    """
    Solver class that takes Command objects from a Pilot and computes the instantaneous efforts necessary.

    model:        a Model object for the multicopter
    kp:           list of proportional gains for the degrees of freedom [world-up, roll, pitch, yaw]
    kd:           list of derivative gains for the degrees of freedom [world-up, roll, pitch, yaw]
    primer_state: State object that will provide initial conditions for yaw, ascent, and time

    """
    def __init__(self, model, kp, kd, primer_state):
        self.model = model
        self.kp = np.array(kp, dtype=np.float64)
        self.kd = np.array(kd, dtype=np.float64)

        if self.model.gmag == 0: self.ascent_dir = np.array([0, 0, 1], dtype=np.float64)
        else: self.ascent_dir = -self.model.gdir

        self.reset(primer_state)
        self.allocator = Allocator(self.model, verbose=False)

    def control(self, state, command):
        """
        Returns the efforts that should be applied to achieve the given pilot commands.

        state:   State object with the current multicopter state
        command: Command object with the pilot's current commands

        """
        # Project body-up direction onto world-up direction (get cosine of the tilt angle)
        ctilt = self.ascent_dir.dot(state.pose.rotate_vector([0, 0, 1]))

        # Don't worry about ascent if completely sideways or upsidedown
        if ctilt <= 1e-5:
            response_force = [0, 0, 0]
        else:
            # Compute actual state's world-coordinate ascent and ascent rate
            state_ascent = self.ascent_dir.dot(state.pose.lin)
            state_ascent_rate = self.ascent_dir.dot(state.pose.rotate_vector(state.twist.lin))
            # Let thruster force be completely along body-up and strong enough to cancel world-coordinate gravity, plus feedback
            response_force = [0, 0, (1/ctilt) * (self.model.mass*self.model.gmag +
                                                 self.kp[0]*(self.ascent - state_ascent) +
                                                 self.kd[0]*(command.ascent_rate - state_ascent_rate))]

        # Ordinary Lie algebraic attitude control scheme for thruster torques
        qdes = motion.quaternion_from_euler(command.roll, command.pitch, self.yaw)
        response_torque = self.kp[1:]*motion.ori_error(qdes, state.pose.ang) + self.kd[1:]*([0, 0, command.yaw_rate] - state.twist.ang)

        # Integrate command rates
        dt = state.time - self.time
        self.yaw = motion.unwrap_angle(self.yaw + dt*command.yaw_rate)
        self.ascent += dt*command.ascent_rate
        self.time += dt

        # Allocate efforts for the response wrench
        return self.allocator.allocate(motion.Wrench(response_force, response_torque))

    def reset(self, state):
        """
        Sets the internally integrated yaw, ascent, and time to the values implicitly contained by a given state.

        state: State object to extract yaw, ascent, and time from

        """
        self.yaw = motion.euler_from_quaternion(state.pose.ang)[2]
        self.ascent = self.ascent_dir.dot(state.pose.lin)
        self.time = state.time
