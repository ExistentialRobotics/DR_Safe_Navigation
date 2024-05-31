#!/usr/bin/env python3
import numpy as np
from numpy import ndarray


class MovObsStates:
    """
    Generate virtual moving obstacle with states [px, py, vx, vy]
    """
    obstacle_state: ndarray
    time: float

    def __init__(self, mo_info, moving_obstacle_eps_dist=0.05):
        """
        INPUT
        @mo_info: moving obstacle parameters including
            p_start         [2d array] 2d initial position [x, y] (m) 
            p_goal          [2d array] 2d goal position [x, y] (m) 
            v               [float] linear velocity (m/s) 
        """

        self.t_restart = 0.0  # timer for moving obstacles start driving backwards
        self.moving_obstacle_eps_dist = moving_obstacle_eps_dist

        self.p_start = mo_info[0]
        self.p_goal = mo_info[1]
        self.v_mo = mo_info[2]

        # compute obstacle heading
        p_delta = self.p_goal - self.p_start
        self.mo_heading = np.arctan2(p_delta[1], p_delta[0])
        self.obstacle_pos2d = mo_info[0]

        pass

    def get_moving_obstacle_state(self, time):
        """
        INPUT
        t               time(s)

        OUTPUT
        s               4d moving obstacle state [px, py, vx, vy]
        """
        self.time = time

        # moving obstacle return after achieved current goal
        self.drive_back()
        t_cur = time - self.t_restart

        # get state vec
        v_x = self.v_mo * np.cos(self.mo_heading)
        v_y = self.v_mo * np.sin(self.mo_heading)
        p_x = self.p_start[0] + v_x * t_cur
        p_y = self.p_start[1] + v_y * t_cur
        p_theta = self.mo_heading

        self.obstacle_pos2d = np.array([p_x, p_y])
        self.obstacle_state = np.array([p_x, p_y, p_theta, v_x, v_y])

        return self.obstacle_state

    def drive_back(self):
        """
        flip goal & start position to drive moving obstacles backwards after goal reached
        """
        p_diff = np.linalg.norm(self.p_goal - self.obstacle_pos2d)
        if p_diff < self.moving_obstacle_eps_dist:
            new_start = self.p_goal
            new_goal = self.p_start
            self.p_start = new_start
            self.p_goal = new_goal
            self.t_restart = self.time
            self.mo_heading += np.pi
        pass
