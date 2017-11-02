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
        - Rotation of the Earth is negligible (no global centripetal or Coriolis effects)
        - Gravity field is uniform over the operating region
        - Translational and angular drag are both approximately linear in twist
        - Same assumptions listed in the Thruster class for thrusters

    """
    def __init__(self, thrusters, mass, inertia, drag_lin, drag_ang, gravity=[0, 0, -9.81]):
        self.thrusters = thrusters
        self.thruster_keys = self.thrusters.keys()  # this list assures the ordering of the thrusters for the B matrices
        self.mass = float(mass)
        self.invmass = 1 / self.mass
        self.inertia = np.array(inertia, dtype=np.float64)
        self.invinertia = npl.inv(self.inertia)
        self.drag_lin = np.array(drag_lin, dtype=np.float64)
        self.drag_ang = np.array(drag_ang, dtype=np.float64)
        self.gravity = np.array(gravity, dtype=np.float64)
        self.B_direcs = np.zeros((3, len(self.thrusters)), dtype=np.float64)
        self.B_levers = np.zeros((3, len(self.thrusters)), dtype=np.float64)
        for i, key in enumerate(self.thruster_keys):
            thr = self.thrusters[key]
            self.B_direcs[:, i] = thr.direction
            self.B_levers[:, i] = np.cross(thr.position, thr.direction)

    def step_dynamics(self, state, efforts, dt, wind_wrench=None):
        """
        Returns a State object containing the multicopter state dt seconds into the future.
        The vector parts of the state are Euler integrated, but the quaternion part is stepped by local linearization.

        state:       State object with the current multicopter state
        efforts:     dictionary of effort commands to each thruster keyed by thruster ID
        wind_wrench: Wrench object for the global wind (external disturbance) in world-coordinates, or None

        """
        state_deriv = self.dynamics(state, efforts, wind_wrench)
        return motion.State(motion.Pose(state.pose.lin + dt*state_deriv.pose_deriv.lin,
                                        motion.quaternion_multiply(state.pose.ang, motion.quaternion_from_rotvec(dt*state_deriv.pose_deriv.ang))),
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
        # Convert efforts to vector of thrusts
        thrusts = np.zeros(len(self.thrusters), dtype=np.float64)
        reactions = np.zeros(len(self.thrusters), dtype=np.float64)
        for i, key in enumerate(self.thruster_keys):
            thr = self.thrusters[key]
            eff = np.clip(efforts[key], thr.min_effort, thr.max_effort)
            thrusts[i] = thr.thrust_from_effort(eff)
            reactions[i] = thr.reaction_from_effort(eff)

        # Sum up thruster forces and torques in body-coordinates
        thr_force = self.B_direcs.dot(thrusts)
        thr_torque = self.B_levers.dot(thrusts) + self.B_direcs.dot(reactions)

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
        twist_lin_deriv = self.invmass * (thr_force + drag_force + grav_force + wind_force) - np.cross(state.twist.ang, state.twist.lin)
        twist_ang_deriv = self.invinertia.dot(thr_torque + drag_torque + wind_torque - np.cross(state.twist.ang, self.inertia.dot(state.twist.ang)))

        # Return a StateDeriv object
        return motion.StateDeriv(motion.Twist(pose_lin_deriv, pose_ang_deriv),
                                 motion.Accel(twist_lin_deriv, twist_ang_deriv),
                                 state.time)
