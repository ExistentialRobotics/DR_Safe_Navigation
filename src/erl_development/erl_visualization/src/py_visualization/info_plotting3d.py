# 3D Plotting Tools

import matplotlib

matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from matplotlib import patches
import numpy as np
import numpy.linalg as LA
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import Axes3D  # Needed for 3D Plotting.
from .quadrotor import Quadrotor


class InfoPlotter3(object):
    # Initialize an Interactive Figure
    def __init__(self, gridmap, plot_num=1, title='Simulator 3D', video=False):
        """
        Construct an Interactive Figure for plotting Information Gathering simulations.
        :param mapmin: The minimum bounds of the map [xmin, ymin].
        :param mapmax: The maximum bounds of the map [xmax, ymax].
        :param cmap: The occupancy grid of the map.
        :param plot_num: The optional plot number.
        :param title: The optional plot title.
        """
        self.plot_num = plot_num
        self.fig = plt.figure(plot_num)
        self.mapmin = gridmap.min()
        self.mapmax = gridmap.max()
        self.cmap = np.array(gridmap.map()).reshape(gridmap.size()[0], -1)
        self.title = title
        self.video = video

        if self.video:
            print("Generating GIF from this Simulation.\n")
        self.images = list()  # For Video Only
        plt.ion()

    def draw_env(self):
        # Plotting Data
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim(self.mapmin[0], self.mapmax[0])
        self.ax.set_ylim(self.mapmin[1], self.mapmax[1])
        self.ax.set_zlim(self.mapmin[2], self.mapmax[2])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title(self.title)
        # Deal with the Grayscale Fact.
        # if self.cmap is not None and len(self.cmap) > 0:  # Plot the CMap only if it is provided
        #     self.ax.imshow(self.cmap, origin='lower', cmap='binary',
        #                    extent=[self.mapmin[0], self.mapmax[0], self.mapmin[1], self.mapmax[1]], zorder=0)

    def draw_point(self, point, clr='b', s=.1):
        """
        Draw a single point
        :param point: The point to draw in 3D.
        :return: None.
        """
        self.ax.scatter(point[0], point[1], point[2], c=clr, s=s)

    def draw_quad(self, state):
        q = Quadrotor()
        q.plot(self.ax, state.position, state.orientation)
