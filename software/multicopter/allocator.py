from __future__ import division
import numpy as np; npl = np.linalg
from scipy.optimize import minimize
import motion


class Allocator(object):
    """
    Solver class for choosing the dictionary of efforts that implement a desired wrench on a multicopter.

    model:   a Model object for the multicopter
    reg:     regularization factor that multiplies the cost of sum(thrusts^2) in the solver
    tol:     solver tolerance, make sure that it is always much less than reg
    method:  string with solver optimization method corresponding to the methods used by scipy.optimize.minimize
    verbose: whether or not verbose error messages should be displayed if optimizer fails

    """
    def __init__(self, model, reg=1e-5, tol=1e-8, method="SLSQP", verbose=False):
        self.model = model
        self.reg = np.float64(reg)
        self.tol = np.float64(tol)
        self.method = str(method)
        self.verbose = bool(verbose)

        # Locally alias a bunch of thruster related things for legibility here
        Bp = self.model.B_direcs
        Bq = self.model.B_levers
        C = np.diag(self.model.reaction_coeffs)
        S = self.reg * np.eye(len(self.model.thrusters))
        J_u = Bp.T.dot(Bp) + Bq.T.dot(Bq) + Bq.T.dot(Bp).dot(C) + C.dot(Bp.T).dot(Bq) + C.dot(Bp.T).dot(Bp).dot(C) + S
        J_ang = Bq + Bp.dot(C)
        J_lin = Bp

        # Construct cost function, Jacobian, and initial guess for thrust solver part of effort allocation
        self.thrust_alloc_cost = lambda thrusts, wrench: 0.5 * ((self.model.wrench_from_thrusts(thrusts) - wrench).norm_squared() + self.reg*np.sum(thrusts**2))
        self.thrust_alloc_cost_jac = lambda thrusts, wrench: thrusts.T.dot(J_u) - wrench.ang.T.dot(J_ang) - wrench.lin.T.dot(J_lin)
        self.thrust_alloc_guess = np.mean(self.model.thrust_limits, axis=1)

    def allocate(self, wrench):
        """
        Returns the optimal allocation of thruster efforts (dictionary) to achieve a desired wrench.

        wrench: Wrench object containing the desired instantaneous force and torque on the multicopter center of mass

        """
        thrust_opt = minimize(fun=self.thrust_alloc_cost, jac=self.thrust_alloc_cost_jac, args=wrench,
                              bounds=self.model.thrust_limits, x0=self.thrust_alloc_guess, method=self.method, tol=self.tol)
        if self.verbose and not thrust_opt.success:
            print "----------"
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
            print "----------"
        efforts = {}
        for i, key in enumerate(self.model.thruster_keys):
            efforts[key] = self.model.thrusters[key].effort_from_thrust(thrust_opt.x[i])
        return efforts
