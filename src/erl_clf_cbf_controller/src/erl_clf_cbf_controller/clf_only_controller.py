#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Reformat on Jul 27 2023


"""

import numpy as np
import cvxpy as cp


class ClfQPController:

    def __init__(self, p1=1e0, p2=1e0, p3=1e2, clf_rate=1.0, cbf_rate=0.25, wheel_offset=0.08, noise_level = 0.01):

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

        # -------- CLF-2 parameters ---------
        self.linear_gain_sq = 0.05
        self.angular_gain_sq = 0.4

        # a user-specified wheel offset (should not be too small > 0.02 or <-0.02)
        self.l = wheel_offset

        # -------- solver status ---------
        self.solve_fail = True
        
        # -------- previous control ----------
        
        self.max_v = 1.2
        
        self.prev_u = np.zeros(2)

        print("Off-wheel: ", self.l)

    def generate_controller(self, rbt_pose, gamma_s, cbf_h_val, cbf_h_grad):
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

        # obtain cbf gradient w.r.t. unicycle system dynamics
       
        # control input
        uu = cp.Variable(3)

        # formulate clf-cbf-qp constraints
        constraints = [
            dVdx.T @ (g_x @ uu[:2]) + self.rateV * V <= uu[2],
            #dot_h[0][0] * uu[0] + dot_h[0][1] * uu[1] + self.rateh * (cbf_h_val - 0.31) >= 0,
            uu[2] >= 0.0,
            cp.abs(uu[0]) <= self.max_v,
            cp.abs(uu[1]) <= 1.00
            #uu[0] >= 0.0
        ]
        
        #print('cbf_h_val:', cbf_h_val)

        # -------- Solver --------
        # formulate objective function

        #obj = cp.Minimize(self.p2 * cp.square(uu[1]) + self.p3 * cp.square(uu[2]))
        
        self.p3 = 5 * (cbf_h_val)

        self.p1 = 2 * (cbf_h_val)
        
        #self.p1 = 1e0

        #obj = cp.Minimize(50 * cp.norm(self.prev_u - uu)**2 + self.p1 * cp.square(uu[0] - 1) + self.p2 * cp.square(uu[1]) + self.p3 * cp.square(delta))
        
        self.p0 = 4
        
        self.p2 = 0.4

        
        
        obj = cp.Minimize(self.p0 * cp.norm(self.prev_u - uu[0:2])**2 + self.p1 * cp.square(uu[0] - self.max_v) + self.p2 * cp.square(uu[1]) + self.p3 * cp.square(uu[2]))


        prob = cp.Problem(obj, constraints)
        
        

        # solving problem
        prob.solve(solver='SCS', verbose=False)

        # Check if the problem was solved successfully
        if prob.status == "infeasible":
            self.solve_fail = True
            print("-------------------------- SOLVER NOT OPTIMAL -------------------------")
            print("[In solver] solver status: ", prob.status)
            print("[In solver] h = ", cbf_h_val)
            print("[In solver] dot_h = ", dot_h)
            self.prev_u = np.array([0.0, 0.0])
            return np.array([0.0, 0.0])
        else:
            self.solve_fail = False
            self.prev_u = uu[0:2].value

        # calculate closed-loop clf value
        self.V_val = V
        self.dotV_val = dVdx.T @ (g_x @ uu.value[0:2])

        return uu.value[0:2]
