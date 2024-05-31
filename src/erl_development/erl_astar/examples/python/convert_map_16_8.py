
from yaml import load, dump
import sys, os

import numpy as np
import matplotlib.pyplot as plt


from pyErlMap import GridMap

if __name__ == "__main__":
    input_yaml = 'maps_8/large_map_2d.yaml'

    node = load(open(input_yaml))

    # Load Parameters from YAML
    start = node['start']
    goal = node['goal']
    epsilon = node['epsilon']
    min = node['mapmin']
    max = node['mapmax']
    res = node['mapres']
    mapdim = node['mapdim']
    storage_order = node['storage']
    mappath = os.path.relpath(input_yaml, os.getcwd()).rsplit("/", 1)[0] + "/" + node['mappath']

    # Load a 16 bit map.
    cmap = np.ndfromtxt(mappath)
    cmap[np.where(cmap == 48)] = 0

    # Now try loading with GridMap Object...
    gridmap = GridMap()
    result = gridmap.load('maps_8/large_map_2d_new.yaml')
    print("result = {}".format(result))
    cmap_gridmap = np.array(gridmap.map()).reshape(gridmap.size()[1], -1)

    fig, axes = plt.subplots(ncols=2)
    axes[0].imshow(cmap, origin='lower', cmap='binary', extent=[min[0], max[0], min[1], max[1]])
    axes[1].imshow(cmap_gridmap, origin='lower', cmap='binary', extent=[min[0], max[0], min[1], max[1]])

    plt.show()