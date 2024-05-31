from pyErlEnv import EnvironmentSE3
from pyErlUtils import SE3Pose
from pyErlMap import GridMap
import numpy as np
import matplotlib.pyplot as plt
from py_visualization import InfoPlotter3

# For SE2
if __name__ == "__main__":
    # Configure Map Size:
    mapmin = [-5, -5, -1]
    mapmax = [5, 5, 3]
    mapres = [.1, .1, .1]

    # Configure Motion Primitives
    #  [x, y, z, r, p, y]
    mprims = [np.array([3, 0, 0, 0, 0, 0]),
              np.array([3, 0, 0, 0, 0, 1]),
              np.array([3, 0, 0, 0, 0, -1])]
    tlist = [1] * 5  # Time discretization.

    # Build Environment
    map = GridMap(mapmin, mapmax, mapres)
    env = EnvironmentSE3(map, mprims, tlist)

    plotter = InfoPlotter3(gridmap=map)
    plotter.draw_env()

    # Set Start coordinate for Plotting some Primitives
    start = [0, 0, 0, 0, 0, 0]
    # Get Successors from Environment.
    succ_list = env.getSuccessors(start)
    print("N_succ=", len(succ_list))

    # Plot Start position.
    # plt.scatter(start[0], start[1], start[2], c='green', s=2)
    # plotter.draw_quad(SE3Pose(start))

    # For each successor primitive, plot the points comprising it's trajectory.
    for succ in succ_list:
        points = np.array(succ.trajectory)
        for point in points:
            plotter.draw_point(point, clr='b', s=.1)


    plt.pause(1000)

