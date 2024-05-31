import sys
import pyErlEnv as ENV  # Environment Package
from pyErlMap import GridMap
import pyAStar as AS  # AStar Package
from yaml import load
import numpy as np
import os
from astar_plotter import AStarPlotter
import pdb
from py_map_data import large_map_3d

from mayavi import mlab # 3D Plotting uses Mayavi

if __name__ == "__main__":
    if len(sys.argv) > 1:
        input_yaml = sys.argv[1]
    else:
        input_yaml = 'examples/maps_8/large_map_3d.yaml'
        # input_yaml = '/home/brent/fla_warehouse1.yaml'

    # Load Map from File
    map = Map(input_yaml)
    map = large_map_3d()
    # Load Map from File
    gridmap = GridMap()
    gridmap.load(input_yaml)

    env = ENV.Environment3D(gridmap)     # Construct Primitive Environment
    plan_env = AS.Planning3D(env, map.goal)     # Construct Planning Environment
    output = AS.AStar3D(map.start, plan_env, epsilon=1.0, log=False)     # Run AStar Planner
    # Convert path to meters
    path = np.array([env.toMetric(point) for point in output.path])
    print(path)
    # Construct Figure
    fig = mlab.figure(1)
    # Plot Map

    filled = map.cmap.reshape(map.mapdim, order='C')
    locations = map.cells2meters(np.argwhere(filled == 1))

    mlab.points3d(locations[:, 0], locations[:, 1], locations[:, 2], locations[:, 2], figure=fig,
                  mode='cube', colormap="cool", scale_factor=.05, scale_mode='none')
    mlab.title('A* Results')
    # Plot Path
    mlab.points3d(path[0, 0], path[0, 1], path[0, 2], figure=fig, mode='sphere', color=(0,1,0), scale_factor=.2) # Start
    mlab.points3d(path[:, 0], path[:, 1], path[:, 2], figure=fig, mode='sphere', color=(0,0,0), scale_factor=.2) # Path
    mlab.points3d(path[-1, 0], path[-1, 1], path[-1, 2], figure=fig, mode='sphere', color=(1,0,0), scale_factor=.2) # Goal

    mlab.show()
