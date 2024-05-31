import sys

from pyErlMap import GridMap
import pyErlEnv as ENV  # Environment Package
import pyAStar as AS  # AStar Package
import numpy as np
import matplotlib.pyplot as plt
from py_visualization import AStarPlotter
from py_map_data import small_map_2d

# Set to True to save a movie of the animation.
make_movie = False

if __name__ == "__main__":

    # Load Map from File
    gridmap = small_map_2d()

    # Bounds on the state space
    max_vel = 10
    max_acc = 4
    max_jrk = 4
    dimmin = [gridmap.min()[0], gridmap.min()[1], -max_vel, -max_vel, -max_acc, -max_acc, -max_jrk, -max_jrk]
    dimmax = [gridmap.max()[0], gridmap.max()[1], max_vel, max_vel, max_acc, max_acc, max_jrk, max_jrk]

    # Setup A* Problem
    epsilon = 1
    start_coord = [0.3, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    goal = [4.2, 4.2]

    env = ENV.EnvironmentFAS2D4O(gridmap, dimmin, dimmax, tau=0.8, rho=2.0)  # Construct environment
    plan_env = AS.PlanningFAS2D4O(env, goal, [1] * 8)  # Construct Planning Environment
    # Run AStar Planner
    output = AS.AStarFAS2D4O(start_coord, plan_env, epsilon=1, log=False)
    path = np.array([env.toMetric(point) for point in output.path])
    traj = env.toTrajectory(output.path, output.action_idx)
    print('Path', path)

    plotter = AStarPlotter(gridmap, title="A* Results")
    plotter.draw_map()
    plotter.plot_start(path)  # Start
    plotter.plot_goal(path)  # Goal
    # Visualize
    # images = plotter.visualize(output, env, batch_size=250, marker=',', state_dim=6, make_movie=make_movie)
    # Plot Path
    plotter.plot_path(path)
    plotter.plot_traj(traj)
    plt.pause(50)

    # Make Movie if desired
    if make_movie:
        map.make_movie(fps=5, filename='data/img/astar_fas.gif')
