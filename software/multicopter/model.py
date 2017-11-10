from __future__ import division
import numpy as np; npl = np.linalg
import motion


class Model(object):
    """
    The system model, including the multicopter itself, its thrusters, and maybe some other hardware?

    thrusters: dictionary of thruster objects keyed by thruster ID
    mass:      scalar total mass
    inertia:   3-by-3-matrix of second moments of inertia
    drag_lin:  translational 1st-order drag coefficients in body-coordinates
    drag_ang:  rotational 1st-order drag coefficients in body-coordinates
    gravity:   local gravitational field vector in world-coordinates

    The following simplifying assumptions are currently made, but they are all very easy to remove later if needed:
        - Translational drag acts at the center of mass
        - Translational and angular drag are both approximately linear in twist
        - Rotation of the Earth is negligible (no global centripetal or Coriolis effects)
        - Gravity field is uniform over the operating region
        - Same assumptions listed in the Thruster class for thrusters

    """
    def __init__(self, thrusters, mass, inertia, drag_lin, drag_ang, gravity=[0, 0, -9.81]):
        self.thrusters = thrusters
        self.mass = np.float64(mass)
        self.inertia = np.array(inertia, dtype=np.float64)
        self.drag_lin = np.array(drag_lin, dtype=np.float64)
        self.drag_ang = np.array(drag_ang, dtype=np.float64)
        self.gravity = np.array(gravity, dtype=np.float64)

        # Memoize these
        self.invmass = 1 / self.mass
        self.invinertia = npl.inv(self.inertia)
        self.gmag = npl.norm(self.gravity)
        if self.gmag != 0: self.gdir = self.gravity / self.gmag
        else: self.gdir = np.zeros(3, dtype=np.float64)

        # Extract thruster information into ordered arrays for consistent and efficient internal use
        self.thruster_keys = self.thrusters.keys()
        self.thruster_list = [self.thrusters[key] for key in self.thruster_keys]
        self.thruster_positions = np.array([thr.position for thr in self.thruster_list])
        self.thrust_limits = np.array([(thr.min_thrust, thr.max_thrust) for thr in self.thruster_list])
        self.reaction_coeffs = np.array([thr.reaction_coeff for thr in self.thruster_list])
        self.B_direcs = np.transpose([thr.direction for thr in self.thruster_list])
        self.B_levers = np.transpose([np.cross(thr.position, thr.direction) for thr in self.thruster_list])

        # Thruster statics
        self.thrusts_from_efforts = lambda efforts: np.clip([self.thrusters[key].thrust_from_effort(efforts[key]) for key in self.thruster_keys],
                                                            self.thrust_limits[:, 0], self.thrust_limits[:, 1])
        self.wrench_from_thrusts = lambda thrusts: motion.Wrench(self.B_direcs.dot(thrusts),
                                                                 self.B_levers.dot(thrusts) + self.B_direcs.dot(self.reaction_coeffs * thrusts))

    def step_dynamics(self, state, efforts, dt, wind_wrench=None):
        """
        Returns a State object containing the multicopter state dt seconds into the future.
        The vector parts of the state are half-Verlet integrated, and the quaternion part is stepped by local linearization.

        state:       State object with the current multicopter state
        efforts:     dictionary of effort commands to each thruster keyed by thruster ID
        wind_wrench: Wrench object for the global wind (external disturbance) in world-coordinates, or None

        """
        state_deriv = self.dynamics(state, efforts, wind_wrench)
        return motion.State(motion.Pose(state.pose.lin + dt*state_deriv.pose_deriv.lin + 0.5*(dt**2)*state.pose.rotate_vector(state_deriv.twist_deriv.lin),
                                        motion.quaternion_multiply(state.pose.ang, motion.quaternion_from_rotvec(dt*state_deriv.pose_deriv.ang + 0.5*(dt**2)*state_deriv.twist_deriv.ang))),
                            motion.Twist(state.twist.lin + dt*state_deriv.twist_deriv.lin,
                                         state.twist.ang + dt*state_deriv.twist_deriv.ang),
                            state.time + dt)

    def dynamics(self, state, efforts, wind_wrench=None):
        """
        Returns the state derivative as a StateDeriv object.
        
        state:       State object with the current multicopter state
        efforts:     dictionary of effort commands to each thruster keyed by thruster ID
        wind_wrench: Wrench object for the global wind (external disturbance) in world-coordinates, or None

        """
        # Sum up thruster forces and torques in body-coordinates
        thr_wrench = self.wrench_from_thrusts(self.thrusts_from_efforts(efforts))

        # Sum up drag force and torque in body-coordinates
        drag_force = -self.drag_lin * state.twist.lin
        drag_torque = -self.drag_ang * state.twist.ang

        # Compute gravity force at current position in body-coordinates
        grav_force = self.mass * state.pose.rotate_vector(self.gravity, reverse=True)

        # Compute the current wind force and torque in body-coordinates
        if wind_wrench is None:
            wind_force = np.zeros(3, dtype=np.float64)
            wind_torque = np.zeros(3, dtype=np.float64)
        else:
            wind_force = state.pose.rotate_vector(wind_wrench.lin, reverse=True)
            wind_torque = state.pose.rotate_vector(wind_wrench.ang, reverse=True)

        # Pose time derivative
        pose_lin_deriv = state.pose.rotate_vector(state.twist.lin)
        pose_ang_deriv = np.copy(state.twist.ang)  # as a member of the Lie algebra

        # Twist time derivative
        twist_lin_deriv = self.invmass * (thr_wrench.lin + drag_force + grav_force + wind_force) - np.cross(state.twist.ang, state.twist.lin)
        twist_ang_deriv = self.invinertia.dot(thr_wrench.ang + drag_torque + wind_torque - np.cross(state.twist.ang, self.inertia.dot(state.twist.ang)))

        # Return a StateDeriv object
        return motion.StateDeriv(motion.Twist(pose_lin_deriv, pose_ang_deriv),
                                 motion.Accel(twist_lin_deriv, twist_ang_deriv),
                                 state.time)
