from typing import Tuple

import numpy as np
from numpy import cos, sin

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D Plotting.


class Quadrotor:
    """
    The Quadrotor class implements a dynamic model for a Quadrotor.
    The state consists of a position, velocity, orientation, and rotational
    velocity. The state vector is
    [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r] which is in R^13.
    """

    def __init__(self, size=0.25, m=0.2, g=9.81, Ixx=1, Iyy=1, Izz=1, noise=0, **kwargs):

        # Quadrotor Intrinsic Parameters
        self.m = m
        self.g = g
        self.I = np.array([Ixx, Iyy, Izz])

        # For Plotting Propellers
        self.p1 = np.array([size / 2, 0, 0, 1]).T
        self.p2 = np.array([-size / 2, 0, 0, 1]).T
        self.p3 = np.array([0, size / 2, 0, 1]).T
        self.p4 = np.array([0, -size / 2, 0, 1]).T
        self.h1 = self.h2 = self.h3 = self.h4 = None

        # Trajectory History
        self.x_data = []
        self.y_data = []
        self.z_data = []
        # super(Quadrotor, self).__init__(13, 2, [], **kwargs)

    def transformation_matrix(self, pos, R):
        '''
        Compute the Transformation Matrix in Homogeneous Coordinates for the Quadrotor.
        Returns:
            A 4x4 Transformation matrix in SE(3).
        '''
        x = pos[0]
        y = pos[1]
        z = pos[2]
        # TODO THIS
        rpy = self.rot2rpy(R)
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        return np.array([[cos(yaw) * cos(pitch),
                          -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
                          sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
                         [sin(yaw) * cos(pitch),
                          cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll),
                          -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
                         [-sin(pitch),
                          cos(pitch) * sin(roll),
                          cos(pitch) * cos(yaw), z]])

    def plot(self, ax, p, R, dt=100):  # pragma: no cover
        '''
        Draw the Quadrotor and Trajectory information on a 3D MPL Axes.
        Args:
            ax: The axes object to plot on.
            dt: The amount of time to pause. If not given, it plots the whole trajectory.

        Returns:
            None.
        '''
        T = self.transformation_matrix(p, R)

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        plot_handles = [self.h1, self.h2, self.h3, self.h4]
        for handle in plot_handles:
            if handle is not None:
                handle[0].remove()

        self.h1 = ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0]],
                          [p1_t[1], p2_t[1], p3_t[1], p4_t[1]],
                          [p1_t[2], p2_t[2], p3_t[2], p4_t[2]], 'k.')

        self.h2 = ax.plot([p1_t[0], p2_t[0]], [p1_t[1], p2_t[1]],
                          [p1_t[2], p2_t[2]], 'r-')
        self.h3 = ax.plot([p3_t[0], p4_t[0]], [p3_t[1], p4_t[1]],
                          [p3_t[2], p4_t[2]], 'r-')

        self.h4 = ax.plot(self.x_data, self.y_data, self.z_data, 'b:')

        plt.pause(dt)


    def rot2rpy(self, R):
        '''
        Computes the roll, pitch, and yaw from a Rotation Matrix R.
        Args:
            R: A rotation matrix in SO(3)

        Returns:
            A vector of roll, pitch , yaw.
        '''

        roll = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], np.hypot(R[2, 1], R[2, 2]))
        yaw = np.arctan2(R[2, 1], R[2, 2])

        return np.array([roll, pitch, yaw])


    def rpy2rot(self, roll, pitch, yaw):
        """
        Calculates the ZYX rotation matrix.
        Args
            Roll: Angular position about the x-axis in radians.
            Pitch: Angular position about the y-axis in radians.
            Yaw: Angular position about the z-axis in radians.
        Returns
            3x3 rotation matrix as NumPy array
        """
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll),
              sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll)],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) *
              sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll)],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw)]
             ])
