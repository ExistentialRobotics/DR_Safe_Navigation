'''
Use this script to take in a 3D map configured from ERL_Mesh_utils, and output a 2D Slice of the map with appropriate
configuration.
'''

import sys, os
sys.path.append('lib')  # Add lib to path
import pyErlMap as UTILS  # Map Utilities Package
from yaml import load
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    if len(sys.argv) > 1:
        input_yaml = sys.argv[1]
    else:
        input_yaml = 'data/simple.yaml'

    # Load Properties from YAML
    yaml_file = load(open(input_yaml))
    mapmin = yaml_file['mapmin']
    mapmax = yaml_file['mapmax']
    mapres = yaml_file['mapres']
    storage = yaml_file['storage']
    mappath = os.path.relpath(input_yaml, os.getcwd()).rsplit("/", 1)[0] + "/" + yaml_file['mappath']

    # Build Map
    map_nd = UTILS.map_nd(mapmin, mapmax, mapres, rowmajor=storage == "rowmajor")  # Default is column major order
    cmap = np.loadtxt(mappath)
    if len(cmap.shape) == 0:  # Handles the case where there is no spacing between the data.
        cmap = np.fromfile(mappath, dtype='b')
        cmap[np.where(cmap == 48)] = 0

    # Reshape into one long vector for slicing.
    cmap = np.asarray(cmap.reshape((1, -1)).astype(np.uint16).squeeze())

    # Get 2D Slice
    map_2d, cmap_2d = map_nd.get2DSlice(cmap, 0)
    cmap_2d = np.asarray(cmap_2d).reshape((map_2d.size[0], map_2d.size[1]))

    # Make Figure
    fig = plt.figure(1)
    ax = fig.subplots()

    # Draw CMAP
    ax.imshow(cmap_2d, origin='lower', cmap='binary', extent=[mapmin[0], mapmax[0], mapmin[1], mapmax[1]])
    plt.show()

