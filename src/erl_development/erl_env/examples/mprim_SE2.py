import sys
import pyErlEnv as Env
from pyErlMap import GridMap
import numpy as np
import matplotlib.pyplot as plt

# For SE2
if __name__ == "__main__":
    # Configure Map Size:
    mapmin = [-10, -10]
    mapmax = [10, 10]
    mapres = [.005, .005]

    # Configure Motion Primitives
    vlist = [1, 1, 1, 1, 1, 3, 3, 3, 3, 3] # Linear Velocities
    wlist = [-2, -1, 0, 1, 2, -2, -1, 0, 1, 2] # Angular Velocities
    tlist = [1] * 10 # Time discretization.

    # Build Environment
    map = GridMap(mapmin, mapmax, mapres)
    env = Env.EnvironmentSE2(map, vlist, wlist, tlist, 50)

    # Set Start coordinate for Plotting some Primitives
    start = [0.0, 0.0, np.pi/2]  # (X, Y)
    # Get Successors from Environment.
    succ_list = env.getSuccessors(start)
    print("N_succ=", len(succ_list))

    # Plot Start position.
    plt.scatter(start[0], start[1], c='green', s=100)
    # For each successor primitive, plot the points comprising it's trajectory.
    for succ in succ_list:
        points = np.array(succ.trajectory)
        plt.scatter(x=points[:, 0], y=points[:, 1], c='blue', s=.1)

    plt.xlim([mapmin[0], mapmax[0]])
    plt.ylim([mapmin[1], mapmax[1]])
    plt.show()
