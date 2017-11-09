from __future__ import division
import numpy as np; npl = np.linalg
from scipy.optimize import minimize
import motion


class Controller(object):
    """
    A multicopter controller.

    model: a Model object for the multicopter
    kp:    list of proportional gains for each degree of freedom [x, y, z, roll, pitch, yaw]
    kd:    list of derivative gains for each degree of freedom [x, y, z, roll, pitch, yaw]

    """
    def __init__(self, model, kp, kd):
        self.model = model
        self.kp = np.array(kp, dtype=np.float64)
        self.kd = np.array(kd, dtype=np.float64)

        # Locally alias a bunch of thruster related things for legibility here
        Bp = self.model.B_direcs
        Bq = self.model.B_levers
        C = np.diag(self.model.reaction_coeffs)
        S = 1e-5*np.eye(len(self.model.thrusters))
        J_u = Bp.T.dot(Bp) + Bq.T.dot(Bq) + Bq.T.dot(Bp).dot(C) + C.dot(Bp.T).dot(Bq) + C.dot(Bp.T).dot(Bp).dot(C) + S
        J_lin = Bp; J_ang = Bq + Bp.dot(C)

        # Construct cost function, Jacobian, and initial guess for thrust solver part of effort allocator
        self.thrust_alloc_cost = lambda thrusts, wrench: 0.5 * ((self.model.wrench_from_thrusts(thrusts) - wrench).norm_squared() + S[0, 0]*np.sum(thrusts**2))
        self.thrust_alloc_cost_jac = lambda thrusts, wrench: thrusts.T.dot(J_u) - wrench.lin.T.dot(J_lin) - wrench.ang.T.dot(J_ang)
        self.thrust_alloc_guess = np.mean(self.model.thrust_limits, axis=1)

        # Function for Lie algebraic orientation error computation
        self.ori_error = lambda qdes, q: motion.rotvec_from_quaternion(motion.quaternion_multiply(motion.quaternion_inverse(q), qdes))

    def effort_allocator(self, wrench):
        """
        Returns the optimal allocation of thruster efforts (dictionary) to achieve a desired wrench.

        wrench: Wrench object containing the desired instantaneous force and torque on the multicopter center of mass

        """
        thrust_opt = minimize(fun=self.thrust_alloc_cost, jac=self.thrust_alloc_cost_jac, args=wrench,
                              bounds=self.model.thrust_limits, x0=self.thrust_alloc_guess, method="SLSQP", tol=1e-8)
        if not thrust_opt.success:
            print "\n----------"
            print "WARNING: Thrust allocator optimization failed."
            print "--"
            print "Wrench requested:"
            print wrench.lin, wrench.ang
            print "Thrust bounds:"
            print self.model.thrust_limits
            print "Initial guess and cost:"
            print self.thrust_alloc_guess, self.thrust_alloc_cost(self.thrust_alloc_guess, wrench)
            print "Thrusts chosen and final cost:"
            print thrust_opt.x, self.thrust_alloc_cost(thrust_opt.x, wrench)
            print "----------\n"
        efforts = {}
        for i, key in enumerate(self.model.thruster_keys):
            efforts[key] = self.model.thrusters[key].effort_from_thrust(thrust_opt.x[i])
        return efforts

    def joy_control(self, state, rpy, ascent, yvel):
        """
        Returns the efforts that should be applied to achieve a specified attitude and ascension rate.
        This is intended for human-in-the-loop control via a joystick.

        state:  the current state of the multicopter in a State object
        rpy:    tuple of roll, pitch and yaw Euler angles in radians defining the desired attitude
        ascent: signed scalar for the desired world z-velocity (altitude rate of change)
        yvel:   signed scalar for the desired yaw velocity in radians per unit time

        """
        ctilt = -self.model.gdir.dot(state.pose.rotate_vector([0, 0, 1]))
        state_ascent = -self.model.gdir.dot(state.pose.rotate_vector(state.twist.lin))
        if np.isclose(ctilt, 0): response_force = [0, 0, 0]
        else: response_force = [0, 0, (self.model.mass*self.model.gmag + self.kd[2]*(ascent - state_ascent)) / ctilt]
        response_torque = self.kp[3:]*self.ori_error(motion.quaternion_from_euler(*rpy), state.pose.ang) + self.kd[3:]*([0, 0, yvel] - state.twist.ang)
        return self.effort_allocator(motion.Wrench(response_force, response_torque))
