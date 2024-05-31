'''

'''
#!/usr/bin/python

import rospy

from py_msg_conversions import _to_cpp, _from_cpp
from erl_msgs.msg import GridMap as GridMapmsg, MultiTargetBelief, PrimitiveTrajectory
from pyErlConversions import gridmapFromROS
from pyErlMap import GridMap, inflateMap2D
import matplotlib.pyplot as plt
import numpy as np


class MapData(object):
    __slots__ = 'gridmap_gt', 'gridmap_online'

    def __init__(self):
        self.gridmap_gt = GridMap()
        self.gridmap_online = GridMap()


# Global Variable to store Map Data
data = MapData()


def map_cb(data, args):
    """
    Callback for subscribing to the GridMap messaging topic. Updates global gridmap (3D) and map_2d (2D) gridmap
    values.
    Args:
        map_msg: The incoming map_msg.

    Returns: None.

    """
    print('Received the GridMap')
    if args[0] == 'gt':
        args[1].gridmap_gt = gridmapFromROS(_to_cpp(data))  # This is an ERL Object
    else:
        args[1].gridmap_online = gridmapFromROS(_to_cpp(data))  # This is an ERL Object


def plot_map_data(ax, map, title='Mapping'):
    if len(map.size()) > 0:
        # Plotting Data
        ax.set_xlim(map.min()[0], map.max()[0])
        ax.set_ylim(map.min()[1], map.max()[1])
        ax.set_xlabel('X axis (m)')
        ax.set_ylabel('Y axis (m)')
        ax.set_title(title)
        # Reshape
        map_2d = map.get2DSlice(10)  # Z cell coordinates of 10.
        cmap = np.array(map_2d.map()).reshape(map_2d.size()[0], -1).astype(np.int8).T
        ax.imshow(cmap, origin='lower', cmap='binary',
                       extent=[map.min()[0], map.max()[0], map.min()[1], map.max()[1]])


if __name__=="__main__":
    rospy.init_node('map_viewer')

    # Setup Subscriber to Map
    map_sub_gt = rospy.Subscriber('map_gt', GridMapmsg, map_cb, ('gt', data))
    map_sub_online = rospy.Subscriber('map_online', GridMapmsg, map_cb, ('o', data))
    print("Waiting for Maps")

    rate = rospy.Rate(1)
    # Setup Matplotlib
    fig, ax = plt.subplots(1, 2)
    plt.ion()
    while not rospy.is_shutdown():
        print("Map data is {}".format(data.gridmap_gt.size()))
        print("Map data is {}".format(data.gridmap_online.size()))
        plot_map_data(ax[0], data.gridmap_gt)
        plot_map_data(ax[1], data.gridmap_online)

        if len(data.gridmap_online.size()) and len(data.gridmap_gt.size()):
            plt.pause(100)
        rate.sleep()
