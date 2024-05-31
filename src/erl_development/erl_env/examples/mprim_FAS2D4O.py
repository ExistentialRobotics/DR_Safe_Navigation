
from pyErlMap import GridMap
import pyErlEnv as Env
import numpy as np
import matplotlib.pyplot as plt

# For FAS (2-D 4th Order (Jerk Controlled))
if __name__ == "__main__":
    # Configure Map Size:
    mapmin = [-10, -10]
    mapmax = [10, 10]
    mapres = [.005, .005]

    # Bounds on the state space
    dimmin = [mapmin[0],  mapmin[1], -4, -4, -4, -4, -4, -4]
    dimmax = [mapmax[0], mapmax[1], 4, 4, 4, 4, 4, 4]

    # Build ENV
    map = GridMap(mapmin, mapmax, mapres)
    env = Env.EnvironmentFAS2D4O(map, dimmin, dimmax, tau=1.0, rho=2.0)

    # Set Start coordinate for Plotting some Primitives
    start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]  # (Position, Velocity, Accel (X,Y))
    start_wp = Env.Waypoint2D4O()
    start_wp.coord = start
    # Get Successors from Environment.
    succ_list = env.getSuccessors(start)
    print("N_succ=", len(succ_list))

    # Plot Start position.
    plt.scatter(start[0], start[1], c='green', s=100)
    tspan = np.linspace(0, 1, 100)
    xspan = []
    yspan = []
    # For each successor primitive, plot the points comprising it's trajectory.
    plt.ion()
    for succ in succ_list:
        coord = succ.coord
        plt.scatter(coord[0], coord[1])
        poly = env.toTrajectorySingle(start_wp, [succ.action_idx])
        for t in tspan:
            xspan.append(poly.scalarValue(t, 0, 0))
            yspan.append(poly.scalarValue(t, 1, 0))
        plt.plot(xspan, yspan)
        xspan = []
        yspan = []
        plt.pause(.01)
    plt.pause(1000)
    plt.xlim([mapmin[0], mapmax[0]])
    plt.ylim([mapmin[1], mapmax[1]])
    plt.show()
