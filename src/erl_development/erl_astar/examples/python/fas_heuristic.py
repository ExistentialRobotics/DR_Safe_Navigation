import pyErlEnv as ENV  # Environment Package
import pyAStar as AS  # AStar Package
import numpy as np
import matplotlib.pyplot as plt
import sys
from py_map_data import small_map_2d


def getHeuristic(plan_env, x,y):
    # return plan_env.getHeuristic(make_wp(x,y))
    return plan_env.getLQMTHeuristic(make_wp(x,y))
    

def make_wp(x,y):
    wp = ENV.Waypoint2D3O()
    wp.coord = [x, y, -1, -1, 0, 0]
    return wp


if __name__ == "__main__":
    # Load Map from File
    gridmap = small_map_2d()

    # FAS dimensions
    max_vel = 10
    max_acc = 4
    dimmin = [gridmap.min()[0], gridmap.min()[1], -max_vel, -max_vel, -max_acc, -max_acc]
    dimmax = [gridmap.max()[0], gridmap.max()[1], max_vel, max_vel, max_acc, max_acc]

    # Goal
    goal = ENV.Waypoint2D3O()
    goal.coord = [4.2, 4.15, 0, 0, 0, 0]
    # Goal Tolerance
    gtol = [2] * 6
    # Build Environments
    env = ENV.EnvironmentFAS2D3O(gridmap, dimmin, dimmax, tau=0.9, rho=10.0)  # Construct environment
    plan_env = AS.PlanningFAS2D3O(env, [4.2, 4.15], [.6] * 6)  # Construct Planning Environment
    # Setup Initial Waypoint

    # Compute LQMT Heuristic
    print('LQMT Heuristic', plan_env.getLQMTHeuristic(goal))
    print('Regular Heuristic', plan_env.getHeuristic(goal))

    # Compute meshgrid
    res = 20
    x = np.linspace(gridmap.min()[0], gridmap.max()[0], res)
    y = np.linspace(gridmap.min()[1], gridmap.max()[1], res)
    xx, yy = np.meshgrid(x,y)

    gen_heatmap = np.vectorize(getHeuristic, excluded='plan_env')
    heatmap = gen_heatmap(plan_env, xx, yy)


    plt.title('Heatmap of Heuristic')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.imshow(heatmap, origin='lower', extent=[gridmap.min()[0], gridmap.max()[0], gridmap.min()[1], gridmap.max()[1]])
    plt.show()

