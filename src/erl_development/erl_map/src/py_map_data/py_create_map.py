'''
This script allows the user to generate a simple map of obstacles by clicking and dragging 2D
rectangles in a gridworld. The maps can then be saved and used for simulating mapping and planning
problems.
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
from pyErlMap import GridMap
import os

root_path = os.path.dirname(os.path.realpath(__file__)) + '/../../data/maps/'


class GridMapCreator(object):
    '''
    GridMapCreator class creates an interactive plot that can be used to generate and
    save simple GridMaps that can be used for robotics problems.
    '''
    def __init__(self):
        """ Initialize the GridMapCreator. """
        self.fig, self.ax = plt.subplots()
        self.text_box = None
        plt.subplots_adjust(bottom=0.2)
        self.gmap = None
        self.xstart = 0
        self.ystart = 0
        self.xend = 0
        self.yend = 0

    def onclick(self, event):
        print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              ('double' if event.dblclick else 'single', event.button,
               event.x, event.y, event.xdata, event.ydata))
        self.xstart = event.xdata
        self.ystart = event.ydata

    def onrelease(self, event):
        print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
              ('double' if event.dblclick else 'single', event.button,
               event.x, event.y, event.xdata, event.ydata))
        self.xend = event.xdata
        self.yend = event.ydata

        self.compute_rectangle()
        plt.draw()

    def save_gmap(self, args):
        """
        Save the GridMap data to a file.
        Returns: The resulting GridMap.
        """
        path = root_path + self.text_box.text
        print("Saving to File {}".format(path))
        self.gmap.save(path)

    def compute_rectangle(self):
        '''
        Create the rectangle and mark it as occupied in the GridMap.
        Returns: None
        '''

        x_ = self.xstart, self.xend
        y_ = self.ystart, self.yend
        self.xmin = min(x_)
        self.xmax = max(x_)
        self.ymin = min(y_)
        self.ymax = max(y_)
        if len(self.gmap.map()) > 0 and \
                self.xstart > self.gmap.min()[0] and self.xend < self.gmap.max()[0] \
                and self.ystart > self.gmap.min()[1] and self.yend < self.gmap.max()[1]:

            start = self.gmap.meters2cells([self.xmin, self.ymin])
            end = self.gmap.meters2cells([self.xmax, self.ymax])
            print("Rectangle: start {} end {}".format(start, end))
            data = np.array(self.gmap.map()).reshape(self.gmap.size()[0], -1)
            data[start[1]:end[1], start[0]:end[0]] = 1
            self.gmap.setMap(data.reshape(data.shape[0] * data.shape[1], -1).flatten())
            self.draw_grid2d()  # Re-draw Rectangle

    def draw_grid2d(self):
        '''
        Draws the current GridMap.
        Returns: None.
        '''
        data = np.array(self.gmap.map()).reshape(self.gmap.size()[0], -1)
        min = self.gmap.min()
        max = self.gmap.max()
        self.ax.imshow(data, origin='lower', cmap='binary',
                  extent=[min[0], max[0], min[1], max[1]])

    def set_axes(self, args):
        '''
        Sets the Axes of the GridMap Drawing.
        Return: None
        '''
        axes = self.ax_box.text # [xmin, xmax, ymin, ymax]
        bounds = [float(iter) for iter in axes[1:-1].split(',')]
        print("Updating Bounds = {}".format(bounds))
        # Update Bounds
        self.ax.set_xlim([bounds[0], bounds[1]])
        self.ax.set_ylim([bounds[2], bounds[3]])

        self.init_gridmap([bounds[0], bounds[2]], [bounds[1], bounds[3]])

    def init_gridmap(self, min, max, res=.1):
        '''
        Initialize a GridMap with the desired data.
        Arg min:
        Arg max:
        Arg res:
        Return None
        '''
        self.gmap = GridMap(min, max, [res, res])
        print("Gridmap shape: {}".format(self.gmap.size()))
        data = np.zeros(self.gmap.size(), dtype=np.bool)
        self.gmap.setMap(data.flatten().tolist())

    def create_map(self):
        '''
        Create a Map and adds event triggers to draw obstacles.
        Returns: None
        '''
        self.gmap = GridMap([0, 0], [1, 1], [.1, .1])
        cid_press = self.fig.canvas.mpl_connect('button_press_event', self.onclick)
        cid_release = self.fig.canvas.mpl_connect('button_release_event', self.onrelease)
        plt.title('GridMap Creator')

        # Create Axis Changing Options
        axbox = plt.axes([0.4, 0.07, 0.3, 0.06])
        axbutton = plt.axes([0.8, 0.07, 0.15, 0.06])
        self.ax_box = TextBox(axbox, 'Axis Limits (xmin, xmax, ymin, ymax)     ', initial="[0, 10, 0, 10]")
        axbutton = Button(axbutton, 'Change Axes')
        axbutton.on_clicked(self.set_axes)

        # Create File Name Options
        filebox = plt.axes([.4, 0.0, 0.3, 0.06])
        self.text_box = TextBox(filebox, 'Map_Name     ', initial="new_map")

        # Create Save Button
        axsave = plt.axes([.85, 0, 0.1, 0.06])
        bsave = Button(axsave, 'Save')
        bsave.on_clicked(self.save_gmap)

        plt.pause(1e10)


def main():
    plt.ion()  # Turn on Interactive Plotting
    creator = GridMapCreator()
    creator.create_map()


if __name__ == "__main__":
    main()