# AStar Plotting Utilities
from yaml import load
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import patches
from datetime import datetime

from pyErlMap import GridMap


class AStarPlotter(object):

    def __init__(self, gridmap, plot_num=1, title='AStar Planner', video=False):
        self.fig = plt.figure(plot_num)
        self.ax = self.fig.gca()
        self.min = gridmap.min()
        self.max = gridmap.max()
        self.res = gridmap.res()
        self.cmap = np.array(gridmap.map()).reshape(gridmap.size()[1], -1)
        self.title = title
        self.video = video
        self.images = []  # For Video Only
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        plt.ion()

    def cells2meters(self, cells):
        return cells * self.res + self.min

    def plot_path(self, path):
        self.ax.scatter(x=path[1:, 0], y=path[1:, 1], color='red', linewidths=.1, s=2, zorder=100)  # Main Path

    def plot_traj(self, traj, samples=5000):
        tspan = np.linspace(traj.start_time(), traj.end_time(), samples)
        xspan = np.zeros(samples)
        yspan = np.zeros(samples)
        for i, t in enumerate(tspan):
            xspan[i] = traj.value(t)[0]  # X Coordinate
            yspan[i] = traj.value(t)[1]  # Y Coordinate
        self.ax.scatter(x=xspan, y=yspan, color='red', linewidths=.1, s=2)

    def plot_start(self, path):
        self.ax.scatter(x=path[0, 0], y=path[0, 1], color='#42f448', linewidths=.5, s=100, zorder=100)  # Start

    def plot_goal(self, path):
        self.ax.scatter(x=path[-1, 0], y=path[-1, 1], color='red', linewidths=.5, s=100, zorder=100)  # End

    def draw_map(self):
        self.ax.imshow(self.cmap, origin='lower', cmap='binary', extent=[self.min[0], self.max[0], self.min[1], self.max[1]])


    def visualize(self, output, env, state_dim, pause=.00001, batch_size=1000, make_movie=False, marker=','):
        plt.ion()  # Set interactive Plot
        self.ax.legend((patches.Patch(facecolor='y'), patches.Patch(facecolor='blue')), ('Open', 'Expanded'))  # Add Legend
        # Generate Visualization
        opened_states = output.opened_list
        closed_states = output.closed_list
        batch_opened = np.empty((0, 2))
        batch_closed = np.empty((0, 2))
        batch_time = datetime.now().microsecond
        # Iterate over both Open and Closed Expansions
        for (closed_key, closed_val) in closed_states.items():
            # Explanation: Closed Key exists for every timestep, but open may not if no new states were opened.
            open_val = opened_states[closed_key] if closed_key in opened_states else []

            # Convert from Internal State representation to Metric
            open_states = np.array([env.toMetric(wp) for wp in open_val])
            closed_states = np.array(env.toMetric(closed_val))
            # Concatenate Batches
            batch_opened = np.concatenate((batch_opened, open_states.reshape((-1, state_dim))[:, 0:2]))
            batch_closed = np.concatenate((batch_closed, closed_states.reshape((-1, state_dim))[:, 0:2]))
            if closed_key % batch_size == 0:
                # Plot current Batch
                self.ax.scatter(x=batch_opened[:, 0], y=batch_opened[:, 1], marker=marker, c='y', alpha=.1, s=100,
                           linewidth=1, zorder=1)
                self.ax.scatter(x=batch_closed[:, 0], y=batch_closed[:, 1], marker=marker, c='b', alpha=.2, s=100,
                           linewidth=.1, zorder=2)
                plt.title('Batch number: ' + str(closed_key / batch_size) + ' Rate (Hz): ' + str(
                    1e5 / (datetime.now().microsecond - batch_time)))
                plt.draw()
                plt.pause(pause)
                # Reset Batch.
                batch_opened = np.empty((0, 2))
                batch_closed = np.empty((0, 2))
                batch_time = datetime.now().microsecond
                if make_movie:
                    image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype='uint8')
                    image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
                    self.images.append(image)
        # End For

        # Plot any remainder of the batch
        self.ax.scatter(x=batch_opened[:, 0], y=batch_opened[:, 1], marker=marker, c='y', alpha=.1, s=100, linewidth=1,
                   zorder=1)
        self.ax.scatter(x=batch_closed[:, 0], y=batch_closed[:, 1], marker=marker, c='b', alpha=.2, s=100, linewidth=.1,
                   zorder=2)
        plt.draw()
        plt.pause(pause)

    def make_movie(self, filename, fps=5):
        # Use Imageio for saving GIFs.
        import imageio
        for i in range(0, 10):
            image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            self.images.append(image)
        kwargs_write = {'fps': fps, 'quantizer': 'nq'}
        imageio.mimsave(filename, self.images, fps=fps)
        print("Saving Movie as ", filename)

