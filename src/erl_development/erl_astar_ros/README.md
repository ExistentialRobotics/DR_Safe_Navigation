# ERL A\* ROS Package

This package is a wrapper around the functionality of the 
ERL A\* package and features ROS functionality for search
based planning for ground and aerial robots.

## ROS Information:

### Nodes:

- erl_astar_2d
    
    This solves 2D Grid A* problems, converting the resulting
    path to a trajectory using Cubic Splines. For quadrotors, it 
    can be configured to generate trajectories with a fixed z-height,
    which is useful for planar flight.        
       
- erl_astar_3d

    This solves 3D Grid A* problems, converting the resulting
    path to a trajectory using Cubic Splines.         

- erl_astar_fas2d3o
    
    This solves A* problems with discretized acceleration inputs.
    The output are 2D trajectories which result directly from the 
    environment. For quadrotors this can be configured to generate
    trajectories with a fixed z height.
    
Each type of Node has same input and output topics.

### Subscribed Topics:
    - odom (nav_msgs/Odometry)
        The odometry of the robot to plan for. Used to find the starting configuration

### Published Topics:
    - traj (erl_msgs/PrimitiveTrajectory)
        The output trajectory generated 

    
The following demos are currently available:

Ground Robot:

    - 2D Grid Planning
    - Fully Actuated System Planning (2D, 3rd Order - Acceleration Control)

Aerial Robot (Quadrotor):

    - 2D Grid Planning (Fixed - Z plane)
    - 3D Grid Planning
    - Fully Actuated System Planning (2D, 3rd Order - Acceleration Control, Fixed Z)

