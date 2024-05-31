#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

@author: kehan
"""

import numpy as np
import cvxpy as cp

import time


class ClfCbfDrccp_dynamic_Controller:

    def __init__(self, p1=1e0, p2=1e0, p3=1e2, clf_rate=1.0, cbf_rate=0.25, wheel_offset=0.1, wasserstein_r = 0.004, epsilon = 0.1, noise_level = 0.01):

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

        # Wasserstein radius and risk tolerance
        self.wasserstein_r = wasserstein_r
        self.epsilon = epsilon

        # maximum linear velo
        self.max_v = 1.2

        # a user-specified wheel offset (should not be too small > 0.02 or <-0.02)
        self.l = wheel_offset

        # -------- CLF-2 parameters ---------
        self.linear_gain_sq = 0.05
        self.angular_gain_sq = 0.4

        # -------- solver status ---------
        self.solve_fail = True

        # -------- previous control ----------
        self.prev_u = np.zeros(2)

    def generate_controller(self, rbt_pose, gamma_s, h_samples, h_grad_samples, dh_dt_samples):
        """
        This function generate control input $v$ and $omega$ for unicycle model using CLF CBF DRCCP Method
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

        # initialize the optimization variables
        N = len(h_samples)
        uu = cp.Variable(2)

        si = cp.Variable(N)
        t = cp.Variable(1)

        delta = cp.Variable()

        # robot_radius = 0.31 + self.l

        robot_radius = 0.31

        # Compute q samples, a vector of dimension 5
        q_samples = [np.hstack([dh_dt_samples[i], h_samples[i] - robot_radius, h_grad_samples[:, i], 0]) for i in
                     range(N)]

        # print('dh_dt_samples:', dh_dt_samples)

        # print('h_samples:', h_samples)

        # print('grad_samples:', h_grad_samples)

        # write F(x)\ubfu = g(x)u for unicycle

        Fu = g_x @ uu

        rateh_vector = np.array([self.rateh])

        rateh_vector = cp.reshape(rateh_vector, (1, 1))

        one_vector = cp.reshape(np.array([1]), (1, 1))

        Fu = cp.reshape(Fu, (3, 1))

        stacked_vector = cp.vstack([one_vector, rateh_vector, Fu])  # Stacks them vertically to make a 5x1 vector.

        ones_vector = np.ones((5, 1))

        # Modify the constraint using the augmented control vectorS
        dro_cbf_constraints = [
            self.wasserstein_r * cp.abs(stacked_vector) / self.epsilon <= (t - (1 / N) * cp.sum(si) / self.epsilon) * ones_vector,
            si >= 0
        ]

        dro_cbf_constraints += [si[i] >= t - stacked_vector.T @ q_samples[i] for i in range(N)]

        # formulate clf-cbf-qp constraints
        additional_constraints = [
            dVdx.T @ Fu + self.rateV * V <= delta,
            # uu[0] >= 0,
            cp.abs(uu[0]) <= self.max_v,
            cp.abs(uu[1]) <= 1.00,
            delta >= 0

        ]

        # -------- Solver --------
        # formulate objective function

        print('h_samples:', h_samples)

        # print('grad samples:', h_grad_samples)

        self.p3 = 5 * (h_samples[0])

        self.p1 = 4 * (h_samples[0])

        self.p0 = 3

        self.p2 = 0.3

        # if (np.abs(self.prev_u[0]) <= 0.001 and np.abs(self.prev_u[1]) <= 0.01):
        #     self.p0 = 1                        
        #     self.p2 = 0.01


        obj = cp.Minimize(
            self.p0 * cp.norm(self.prev_u - uu) ** 2 + self.p1 * cp.square(uu[0] - self.max_v) + self.p2 * cp.square(
                uu[1]) + self.p3 * cp.square(delta))

        constraints = dro_cbf_constraints + additional_constraints

        # constraints = additional_constraints

        prob = cp.Problem(obj, constraints)

        start_time = time.time()

        # solving problem
        # prob.solve(solver='SCS', verbose=False, max_iters=140000, eps = 1e-4)

        prob.solve(solver='SCS', verbose=False)

        solver_time = time.time() - start_time

        self.prev_u = uu.value

        # print('self_p2:', self.p2)

        # print('relax:', delta.value)

        # if prob.status == 'infeasible':
        #     return np.array([0.0, 0.0]), False, solver_time

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