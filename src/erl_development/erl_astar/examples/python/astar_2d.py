
import pyErlEnv as ENV  # Environment Package
from pyAStar import Planning2D, AStar2D # AStar Package
import numpy as np
import matplotlib.pyplot as plt
from py_visualization import AStarPlotter
from py_map_data import large_map_2d

# Set to True to save a movie of the animation.
make_movie = False

if __name__ == "__main__":

    # Load Map from File
    gridmap = large_map_2d()

    # Setup an AStar Problem
    start = [-20.95, 20.8]
    goal = [6.4, -14.3]
    epsilon = 1.0

    plotter = AStarPlotter(gridmap, title="A* Results")

    env = ENV.Environment2D(gridmap)     # Construct Primitive Environment
    plan_env = Planning2D(env, goal)     # Construct Planning Environment
    output = AStar2D(start, plan_env, epsilon=1.0, log=True)     # Run AStar Planner

    # Convert path to meters
    path = np.array([env.toMetric(point) for point in output.path])

    # Visualize Solution.
    plotter.draw_map()
    plotter.plot_start(path)  # Start
    plotter.plot_goal(path)  # Goal
    # Make Visualization
    plotter.visualize(output, env,  state_dim=2, pause=.0001, batch_size=1000, marker='.', make_movie=make_movie)
    plotter.plot_path(path)      # Plot Path
    plt.pause(1000)

    if make_movie:
        plotter.make_movie(filename='data/img/astar2d.gif', fps=5)

