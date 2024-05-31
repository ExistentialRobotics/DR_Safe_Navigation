import sys
import pyErlEnv as Env
from pyErlMap import GridMap
import numpy as np
import matplotlib.pyplot as plt

# For 2D
if __name__ == "__main__":
    # Configure Map Size:
    mapmin = [-10, -10]
    mapmax = [10, 10]
    mapres = [1, 1]

    # Build Environment
    map = GridMap(mapmin, mapmax, mapres)
    env = Env.Environment2D(map)

    # Set Start coordinate for Plotting some Primitives
    start = [0.0, 0.0]  # (X, Y)
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
