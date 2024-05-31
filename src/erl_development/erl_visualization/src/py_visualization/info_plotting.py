# 2D Plotting Tools

import matplotlib

matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from matplotlib import patches
import numpy as np
import numpy.linalg as LA
from matplotlib.lines import Line2D


class InfoPlotter(object):

    # Initialize an Interactive Figure
    def __init__(self, gridmap, plot_num=1, title='Simulator', video=False):
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

    def set_title(self, title=None):
        if title is not None:
            self.fig.gca().set_title(title)

    def save_fig(self, filename):
        # self.fig.set_size_inches(18.5, 10.5)
        self.fig.savefig(filename,  bbox_inches='tight')

    def update_gridmap(self, gridmap):
        """
        Update the internal gridmap with a new one.
        :param gridmap: The gridmap to update.
        :return: None.
        """
        self.mapmin = gridmap.min()
        self.mapmax = gridmap.max()
        self.cmap = np.array(gridmap.map()).reshape(gridmap.size()[0], -1)
        if self.cmap is not None and len(self.cmap) > 0:  # Plot the CMap only if it is provided
            self.ax.imshow(self.cmap, origin='lower', cmap='binary',
                           extent=[self.mapmin[0], self.mapmax[0], self.mapmin[1], self.mapmax[1]], zorder=0)

    def draw_env(self):
        """
        Draw the environment according to the map parameters and occupancy grid.
        :return: The resulting environment.
        """
        # Plotting Data
        self.ax = self.fig.subplots()
        self.ax.set_xlim(self.mapmin[0], self.mapmax[0])
        self.ax.set_ylim(self.mapmin[1], self.mapmax[1])
        self.ax.set_xlabel('X axis (m)')
        self.ax.set_ylabel('Y axis (m)')
        self.ax.set_title(self.title)
        # Deal with the Grayscale Fact.
        if self.cmap is not None and len(self.cmap) > 0:  # Plot the CMap only if it is provided
            self.ax.imshow(self.cmap, origin='lower', cmap='binary',
                           extent=[self.mapmin[0], self.mapmax[0], self.mapmin[1], self.mapmax[1]], zorder=0)

    def draw_heatmap(self, heatmap):
        """
        Draws a heatmap on the environment bounds.
        :param heatmap: The heatmap to display.
        :return:
        """
        hm = self.ax.imshow(heatmap, origin='lower', cmap='inferno',
                            extent=[self.mapmin[0], self.mapmax[0], self.mapmin[1], self.mapmax[1]], zorder=1)
        self.fig.colorbar(hm)
        return self.ax

    def draw_robot(self, pose, clr='b', size=1, zorder=5):
        """
        Draw a Robot as a triangle with some heading.
        :param pose: The pose of the robot (x,y,theta).
        :param clr: The color to plot the oriented triangle.
        :param size: The size of the patch.
        :return: The resulting patch.
        """
        x = pose[0]  # Get Pose Information.
        y = pose[1]
        th = pose[2]
        if (len(pose) == 6): # This is for full SE(3)
            th = pose[-1]
        # Construct Triangle Polygon.
        XY = np.array([[x + size * np.cos(th), x + size * np.cos(th - 2.7), x + size * np.cos(th + 2.7)],
                       [y + size * np.sin(th), y + size * np.sin(th - 2.7), y + size * np.sin(th + 2.7)]]).transpose()
        return self.ax.add_patch(patches.Polygon(XY, facecolor=clr, zorder=zorder))

    def draw_point(self, point, clr='b'):
        """
        Draw a single point
        :param point: The point to draw in 2D.
        :return: None.
        """
        self.ax.scatter(point[0], point[1], c=clr)

    def draw_fov(self, pose, range, fov, clr='c'):
        """
        Draw a wedge field-of-view around the robot.
        :param pose: The pose to center the wedge around.
        :param range: The maximum sensing range.
        :param fov: The field of view in degrees.
        :param clr: The color to plot the wedge.
        :return: The resulting patch added.
        """
        # Construct Wedge
        x = pose[0]
        y = pose[1]
        th = pose[2]
        if len(pose) == 6: # This is for full SE(3)
            th = pose[-1]
        # Convert Theta to Degrees
        th_deg = 180 / np.pi * th
        theta1 = th_deg - fov / 2
        theta2 = th_deg + fov / 2
        return self.ax.add_patch(patches.Wedge((x, y), range, theta1, theta2,
                                               fill=True,
                                               edgecolor='b',
                                               linewidth=1.5,
                                               linestyle='--',
                                               facecolor=clr,
                                               alpha=.4, zorder=3))

    def draw_camera(self, pose, range, fov, clr='b', zorder=5):
        """
        Draw a Camera field of view from a given pose as a triangular field of view.
        :param pose: The pose of the robot (x,y,theta).
        :param clr: The color to plot the oriented triangle.
        :param size: The size of the patch.
        :return: The resulting patch.
        """
        x = pose[0]  # Get Pose Information.
        y = pose[1]
        th = pose[2]
        if (len(pose) == 6): # This is for full SE(3)
            th = pose[-1]
        # Construct Triangle Polygon.
        fov = fov * np.pi / 180
        XY = np.array([[x, x + range * np.cos(th - fov/2), x + range * np.cos(th + fov / 2)],
                       [y, y + range * np.sin(th - fov/2), y + range * np.sin(th + fov/2)]]).transpose()
        return self.ax.add_patch(patches.Polygon(XY, facecolor=clr, zorder=zorder, alpha=.2))

    def draw_laser(self, pose, sensor, scan, clr='r', lines=True, linewidth=.1):
        """
        Draw a laser scan from the given pose.
        :param pose: The pose where the scan originates.
        :param sensor: The sensor for characteristics.
        :param scan: The generated scan data.
        :return: None.
        """
        # Angle Increments.
        min_hang = sensor.min_hang
        max_hang = sensor.max_hang
        beam_count = len(scan.z)
        angle_inc = (max_hang - min_hang) / (beam_count -1)
        end_pts = []
        th = pose[2]
        if len(pose) == 6: # This is for full SE(3)
            th = pose[-1]
        for i in range(0, beam_count):
            ang = min_hang + i * angle_inc + th
            end_pts.append([scan.z[i] * np.cos(ang) + pose[0], scan.z[i] * np.sin(ang) + pose[1]])
        if lines:
            for end_pt in end_pts:
                self.ax.add_line(Line2D([pose[0], end_pt[0]], [pose[1], end_pt[1]], c=clr,
                                        linewidth=linewidth,  zorder=1))
        else:
            end_pts = np.array(end_pts)
            self.ax.scatter(x=end_pts[:, 0], y=end_pts[:, 1], c=clr)

    def draw_observed_points(self, path, range, fov, clr='c'):
        """
        Draws the sequence of observable spaces from a path.
        :param path: The sequence of poses to observe from.
        :param range: The range to plot each observation.
        :param fov: The field of view for each observation.
        :param clr: The color to plot in.
        :return: None
        """
        for pose in path:
            position = pose.position
            yaw = pose.getYaw()
            self.draw_fov([position[0], position[1], yaw], range, fov, clr)

    def draw_expanded_nodes(self, nodes, clr='y'):
        """
        Draw the set of expanded nodes from a search query.
        :param nodes: The set of nodes.
        :param clr: The color to plot.
        :return: None
        """
        nodes_arr = np.array([[state.position[0], state.position[1]] for idx, state in nodes.items()])
        self.ax.scatter(nodes_arr[:, 0], nodes_arr[:, 1], c=clr, marker='.')

    def draw_cov(self, pose, cov, clr='y', confidence=0.99, zorder=2):
        """
        Draw a covariance ellipse around a particular pose.
        :param pose: The center of the ellipse.
        :param cov: The covariance matrix to plot.
        :param clr: The color to plot.
        :param confidence: The confidence on [0, 1].
        :return: The resulting patch added to the figure.
        """
        # For 2D confidence, we can use the inverse of Chi-Square with 2 DOF
        s = -2 * np.log(1 - confidence)  # Confidence Parameter: s
        eig_val, eig_vec = LA.eig(cov)
        if (2 * np.sqrt(s * eig_val[0] > 2 or 2 * np.sqrt(s * eig_val[1]) > 2)):
            s = s/8
        ellipse = patches.Ellipse(pose, 2 * np.sqrt(s * eig_val[0]), 2 * np.sqrt(s * eig_val[1]),
                                  angle=180 / np.pi * np.arctan(eig_vec[0][1] / eig_vec[0][0]),
                                  fill=True, zorder=zorder,
                                  facecolor=clr, alpha=0.1)
        return self.ax.add_patch(ellipse)

    def draw_paths(self, paths, clr='y', zorder=5, yaw=False):
        """
        Draw a set of paths on the map.
        :param paths: The set of paths to be drawn.
        :param clr: The color to draw the paths.
        :return: None.
        """
        # print('Size of paths: ', max([len(path) for path in paths]))
        for path in paths:
            if len(path) > 0:
                data = np.array([[state.position[0], state.position[1], state.getYaw()] for state in path])
                self.ax.plot(data[:, 0], data[:, 1], c=clr, marker=',', zorder=zorder)
                if yaw:
                    for i in range(0, data.shape[0]):
                        self.draw_robot([data[i, 0], data[i, 1], data[i, 2]], clr=clr, size=.2)


    def draw_target_path(self, path, clr='r', zorder=10):
        """
        Draw a predicted target path on the map.
        :param path: The predicted target path.
        :param clr: The color to draw.
        :param zorder: The height of the path.
        :return: None
        """
        self.ax.scatter(path[0, :, 0], path[0, :, 1], c=clr, marker='.', zorder=zorder)

    def clear_plot(self):
        """
        Clear the figure between timesteps.
        :return: The resulting cleared figure.
        """
        self.fig.clf()

    def plot_target_cov(self, mean, cov, dim=2):
        '''
        Draw the mean and covariance from Targets Given a block mean and covariance matrix.
        Args:
            mean: The target mean.
            cov: The target covariance.
            dim: The dimension of each target.

        Returns: None
        '''
        index = 0
        for i in range(0, np.shape(mean)[0] / dim): # N_targets
            self.draw_cov(mean[index:index+dim], cov[index:index+dim, index:index+dim], clr='r')
            index += dim

    def plot_state(self, robots, attacks=None, robot_size=0.5, SensingRange=8, fov=360, targets=[], target_path=None):
        """
        Plot the simulation state of the set of robots.
        :param robots: The set of robots.
        :param attacks: The list of whether robots are attacked.
        :param robot_size: The markersize to plot each robot.
        :param SensingRange: Sensing range for each robot.
        :param fov: The field of view for each robot.
        :param targets: The set of targets to plot.
        :return: None
        """
        self.clear_plot()  # Refresh Plot
        self.draw_env()
        if attacks is None:
            attacks = [0] * len(robots)
        # Plot Robot Positions
        for idx, robot in enumerate(robots):
            pose = robot.getState().getSE2()  # (X, Y, Yaw)
            color = 'b' if not attacks[idx] else 'r'
            self.draw_robot(pose, size=robot_size, clr=color)
            self.draw_fov(pose, SensingRange, fov, clr=color)  # TODO Get these parameters from sensor itself

            if hasattr(robot.filter, 'belief_map'):
                for target in robot.filter.belief_map.values():
                    mean = target.getState()
                    cov = target.getCovariance()[0:2, 0:2]
                    self.draw_cov(mean, cov)
            if hasattr(robot.filter, 'target') and robot.filter.target is not None:
                mean = robot.filter.target.getState()
                cov = robot.filter.getCovariance()[0:2, 0:2]
                self.draw_cov(mean, cov)

        predicted_traj = np.array(target_path)
        for idx, ID in enumerate(targets):
            true_target = targets[ID]
            true_target_pose = true_target.getState()[:2]
            self.ax.plot(true_target_pose[0], true_target_pose[1], marker='o', markersize=8, linestyle='None',
                         markerfacecolor='r', markeredgecolor='r', zorder=5)

            if target_path is not None:
                self.ax.plot(predicted_traj[:, 4 * idx], predicted_traj[:, 4 * idx + 1], c='r')
        plt.draw()

    def plot_targets_cov(self, belief_map):
        for target in belief_map.values():
            mean = target.getState()
            cov = target.getCovariance()[0:2, 0:2]
            self.draw_cov(mean, cov)

    def cache_frame(self):
        if self.video:
            print("Caching Frame. Images with size {}\n".format(len(self.images)))
            self.fig.canvas.draw()
            image = np.frombuffer(self.fig.canvas.tostring_rgb(), dtype='uint8')
            image = image.reshape(self.fig.canvas.get_width_height()[::-1] + (3,))
            self.images.append(image)

    # End function
    def add_legend(self, handles=None):
        if handles is None:
            target = patches.Patch(color='r', label='Target')
            cov = patches.Patch(color='g', label='Covariance')
            fov = Line2D([0], [0], linestyle='--', color='c', label='Sensor Range')
            exp = patches.Patch(color='y', label='Expanded State')
            path = Line2D([0], [0], marker='o', color='b', label='Robot Path')
            handles = [target, cov, fov, exp, path]
            # Target
        self.ax.legend(handles=handles, loc='upper right', prop={'size': 6})

    def save_video(self, filename, fps=10):
        """
        Saves a recorded GIF to a file if the option was made available in the plotter construction.
        :param filename: The file to save to.
        :param fps: The framerate to record.
        :return: None
        """
        import imageio
        import cv2
        import scipy.misc
        kwargs_write = {'fps': fps, 'quantizer': 'nq'}
        print("Saving Video with {} images at.".format(len(self.images), filename))
        imageio.mimsave('./' + filename + '.gif', self.images, fps=fps)
        path = './' + filename + '/data'
        import os
        if not os.path.exists(path):
            os.makedirs(path)
        for idx, image in enumerate(self.images):
            scipy.misc.imsave(path + '_' + str(idx)+'.jpg', image)
            # cv2.imwrite(, image)


    def save_cmap(self, filename='data/maps/emptySmall/obstacles.cfg'):
        # If cols not odd, append a column of zeros
        cmap = self.cmap.astype(np.uint16)
        print('CMap shape:', cmap.shape)
        print('Cmap', cmap)
        if cmap.shape[1] % 2 is 0:
            cmap = np.concatenate((cmap, np.zeros((cmap.shape[0], 1), dtype=np.uint16)), axis=1)
        if cmap.shape[0] % 2 is 0:
            cmap = np.concatenate((cmap, np.zeros((1, cmap.shape[1]), dtype=np.uint16)))
        np.savetxt(fname=filename, X=cmap, fmt='%d')


if __name__ == '__main__':
    filename = 'data/maps/emptySmall/obstacles.cfg'
    data = np.loadtxt(filename)
    print('Original: ', data)
    print('Shape: ', data.shape)
    import scipy.ndimage

    result = scipy.ndimage.zoom(data, 20, order=0)
    shave = 10
    result = result[shave - 1:-shave, shave - 1:-shave]
    print('Original: ', result)
    print('Shape: ', result.shape)
    np.savetxt(fname='data/maps/emptySmall/obstacles_large.cfg', X=result, fmt='%d')
    plt.imshow(result)
    plt.show()
