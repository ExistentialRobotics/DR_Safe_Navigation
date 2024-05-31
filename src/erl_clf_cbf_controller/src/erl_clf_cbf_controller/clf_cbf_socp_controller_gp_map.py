#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: kehan
"""

import numpy as np
import cvxpy as cp


class ClfCbfSOCP_GP_MAP:

    def __init__(self, p1=1e0, p2=1e0, p3=1e1, clf_rate=1.0, cbf_rate=0.3, wheel_offset=0.08, epsilon=0.2):

        # -------- optimizer parameters ---------
        # linear velocity penalty
        self.p1 = p1
        # angular velocity penalty
        self.p2 = p2
        # CLF penalty (for CLF-CBF-QP)
        self.p3 = p3
        # CLF decay rate
        self.rateV = clf_rate
        # CBF decay rate
        self.rateh = cbf_rate

        self.max_v = 1.2

        # -------- CLF-2 parameters ---------

        self.linear_gain_sq = 0.05
        self.angular_gain_sq = 0.4

        # a user-specified wheel offset (should not be too small > 0.02 or <-0.02)
        self.l = wheel_offset

        # risk tolerance
        self.risk_tolerance = 1 - epsilon

        # -------- solver status ---------
        self.solve_fail = True

        # -------- previous control ----------

        self.prev_u = np.zeros(2)

    def generate_controller(self, rbt_pose, gamma_s, cbf_h_val, cbf_h_grad, cbf_h_variance, cbf_h_grad_var):
        """
        This function generate control input $v$ and $omega$ for unicycle model using CLF-CBF-QP method
        dot{x} = [[cos(theta), 0],
                  [sin(theta), 0]
                  [0, 1]] * [[v], [omega]]
        """
        # -------- CLF 1 --------
        # V = np.linalg.norm(rbt_pose[0:2] - gamma_s) ** 2
        # dVdx = np.append(2 * (rbt_pose[0:2] - gamma_s), 0)

        # -------- Clf 2 ---------
        proj_perp = -np.array([[-np.sin(rbt_pose[2])],
                               [np.cos(rbt_pose[2])]])

        proj_alog = -np.array([[np.cos(rbt_pose[2])],
                               [np.sin(rbt_pose[2])]])

        m_z = proj_perp.T @ (rbt_pose[0:2] - gamma_s[0:2])

        n_z = proj_alog.T @ (rbt_pose[0:2] - gamma_s[0:2])

        V = 0.5 * (self.linear_gain_sq * ((rbt_pose[0] - gamma_s[0]) ** 2 + (rbt_pose[1] - gamma_s[1]) ** 2) +
                   self.angular_gain_sq * np.arctan2(m_z, n_z) ** 2)

        dV_dxy = (self.linear_gain_sq * (rbt_pose[0:2] - gamma_s[0:2]).T +
                  self.angular_gain_sq * np.arctan2(m_z, n_z) * (1 / (m_z ** 2 + n_z ** 2)) *
                  (n_z * proj_perp.T - m_z * proj_alog.T))

        dV_dtheta = -self.angular_gain_sq * np.arctan2(m_z, n_z)

        dVdx = np.row_stack((dV_dxy[0, 0], dV_dxy[0, 1], dV_dtheta))

        # -------- unicycle system dynamics ---------
        g_x = np.array([[np.cos(rbt_pose[2]), -self.l * np.sin(rbt_pose[2])],
                        [np.sin(rbt_pose[2]), self.l * np.cos(rbt_pose[2])],
                        [0, 1]])

        # obtain cbf gradient w.r.t. unicycle system dynamics using provided GP properties

        dot_h = np.array([[np.cos(rbt_pose[2]) * cbf_h_grad[0] + np.sin(rbt_pose[2]) * cbf_h_grad[1],
                           -self.l * np.sin(rbt_pose[2]) * cbf_h_grad[0] + self.l * np.cos(rbt_pose[2]) * cbf_h_grad[
                               1]]])

        c_p = np.sqrt(self.risk_tolerance / (1 - self.risk_tolerance))

        print('-------------variance_h---------------:', cbf_h_variance)

        #print('-------------grad_h---------------:', cbf_h_grad)

        #print('-------------grad_var---------------:', cbf_h_grad_var)

        # grad_hx = np.array([[cbf_h_grad[0][0], cbf_h_grad[1][0], 0]])
        #
        #grad_h_var = np.array([[cbf_h_grad_var[0][0], cbf_h_grad_var[1][0], 0]])

        grad_h_var_matrix = np.zeros((3, 3))

        grad_h_var_matrix[0, 0] = cbf_h_grad_var[0][0]
        grad_h_var_matrix[1, 1] = cbf_h_grad_var[1][0]
        grad_h_var_matrix[2, 2] = 0

        # Calculate Cov[Px]
        cov_Px = np.zeros((3, 3))
        cov_Px[0, 0] = (self.rateh ** 2) * cbf_h_variance 
        #cov_Px[0, 1:] = self.rateh * np.matmul(grad_h_var, g_x).flatten()
        #cov_Px[1:, 0] = cov_Px[0, 1:]
        cov_Px[1:, 1:] = np.matmul(np.matmul(g_x.T, grad_h_var_matrix), g_x)

        #print('var_px:', cov_Px)

        # det_cov_Px = np.linalg.det(cov_Px)
        # eigenvalues_cov_Px = np.linalg.eigvals(cov_Px)
        # smallest_eigenvalue = np.min(eigenvalues_cov_Px)

        # print("Determinant of cov_Px:", det_cov_Px)
        # print("Smallest eigenvalue of cov_Px:", smallest_eigenvalue)

        # SOCP variable for CBF constraint
        t = cp.Variable()

        # control input
        uu = cp.Variable(2)

        delta = cp.Variable()

        self.p3 = 5 * (cbf_h_val[0])

        self.p1 = 4 * (cbf_h_val[0])

        self.p0 = 1

        self.p2 = 0.3

        # to deal with some corner case
        # if (np.abs(self.prev_u[0]) <= 0.001 and np.abs(self.prev_u[1]) <= 0.01):
        #     self.p0 = 1
        #     self.p2 = 0.01

        # formulate clf-cbf-gp socp constraints
        constraints = [
            dVdx.T @ (g_x @ uu[:2]) + self.rateV * V <= delta,
            cp.SOC(t, cp.hstack([self.p0 * (uu[0] - self.prev_u[0]), self.p0 * (uu[1] - self.prev_u[1]),
                                 self.p1 * (uu[0] - self.max_v), self.p2 * uu[1], self.p3 * delta])),
            # SOC objective constraint
            # uu[2] >= 0,
            # cp.SOC(t, cp.hstack([uu[0] - self.prev_u[0], uu[1] - self.prev_u[1], self.p2 * uu[1], self.p3 * delta])),
            cp.abs(uu[0]) <= self.max_v,
            cp.abs(uu[1]) <= 1.00,
            #uu[0] >= 0
        ]

        # Given cov_Px, compute D(x) using Cholesky decomposition
        # Regularize cov_Px slightly to ensure it's PD
        #epsilon = 1e-6
        #cov_Px += epsilon * np.eye(3)

        # Compute D(x) using Cholesky decomposition
        D_x = np.linalg.cholesky(cov_Px)

        # SOC constraint for the safety requirement
        LHS = dot_h[0][0] * uu[0] + dot_h[0][1] * uu[1] + self.rateh * (cbf_h_val - 0.31)

        u_underline = cp.vstack([1, uu[0], uu[1]])

        print('------------barrier value-----------:', cbf_h_val - 0.31)

        # constraints.append(cp.SOC(LHS, c_p * D_x @ u_underline))

        # constraints.append(cp.SOC(LHS, c_p * D_x[0,0]))

        # ---------------temporarily use the following constraint, not sure about Hessian computation ------------
        constraints.append(LHS - c_p * cp.norm(D_x @ u_underline) >= 0)

        # -------- Solver --------
        # formulate objective function
        obj = cp.Minimize(t)

        prob = cp.Problem(obj, constraints)

        # solving problem
        prob.solve(solver='SCS', verbose=False)

        # Check if the problem was solved successfully
        if prob.status != 'optimal':
            self.solve_fail = True
            print("-------------------------- SOLVER NOT OPTIMAL -------------------------")
            print("[In solver] solver status: ", prob.status)

            self.prev_u = np.array([0.0, 0.0])

            return np.array([0.0, 0.0])
        else:
            self.solve_fail = False

        # calculate closed-loop clf value
        self.V_val = V
        self.dotV_val = dVdx.T @ (g_x @ uu.value)

        return uu.value[0:2]