import pyErlEnv as ENV  # Environment Package
import pyAStar as AS  # AStar Package
import numpy as np
import matplotlib.pyplot as plt
from py_visualization import AStarPlotter
from py_map_data import circles_map_se2


make_movie = False

if __name__ == "__main__":

    # Load Map from File
    gridmap = circles_map_se2()
    start = [10, 90, 0]
    goal = [50, 1, 0]
    tolerance = [2, 2, np.pi/4]
    epsilon = 2

    # Setup Action Space.
    vlist = [1, 1, 1, 3, 3, 3]
    wlist = [0, -1, 1, 0, -1, 1]
    tlist = [1] * 6
    env = ENV.EnvironmentSE2(gridmap, vlist, wlist, tlist, 25)     # Construct Primitive Environment
    plan_env = AS.PlanningSE2(env, goal, tolerance)     # Construct Planning Environment
    print('Starting A* Planner')
    output = AS.AStarSE2(start, plan_env, epsilon=1.0, log=True)     # Run AStar Planner
    path = np.array([env.toMetric(point) for point in output.path])

    plotter = AStarPlotter(gridmap, title="A* Results")
    plotter.draw_map()
    plotter.plot_start(path)  # Start
    plotter.plot_goal(path)  # Goal
    # Visualize
    # images = plotter.visualize(output, env, batch_size=250, marker=',', state_dim=6, make_movie=make_movie)
    # Plot Path
    plotter.plot_path(path)
    plt.pause(50)

    if make_movie:
        map.make_movie(fps=5, filename='data/img/astar_se2.gif')
