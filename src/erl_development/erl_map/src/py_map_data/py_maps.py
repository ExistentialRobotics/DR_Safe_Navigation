'''This file allows easy importing of various maps across different packages while storing them in one place.'''
from pyErlMap import GridMap
import os

root_path = os.path.dirname(os.path.realpath(__file__)) + '/../../data/maps/'


def loader(path, debug=False):
    if debug:
        print("Loading a GridMap from {}".format(path))
    path = root_path + path
    gridmap = GridMap()
    gridmap.load(path)
    return gridmap


def obstacle_map(debug=False):
    return loader('obstacles.yaml', debug)


def large_map_2d(debug=False):
    return loader('large_map_2d.yaml', debug)


def circles_map_se2(debug=False):
    return loader('circles_map_se2.yaml', debug)


def large_map_3d(debug=False):
    return loader('large_map_3d.yaml', debug)


def small_map_2d(debug=False):
    return loader('small_map_2d.yaml', debug)


def load_map(map_name, debug=False):
    return loader(map_name + '.yaml', debug)