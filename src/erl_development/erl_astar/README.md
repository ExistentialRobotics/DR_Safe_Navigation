# A* planning package
  + This is an updated version of https://bitbucket.org/ExistentialRobotics/nx_astar. This package features an implementation of the A* search-based planning algorithm, for 
  a variety of environments provided in the [erl_env](https://bitbucket.org/ExistentialRobotics/erl_env) 
  package. The environments include a 2D and 3D grid, SE(2) with motion primitives discretizing linear and angular velocity, and lastly
  a fully actuated system in 2D with discretized jerk primitives.

![](data/img/astar2d.gif)

## Table of Contents:

- [Compilation](#compilation)
- [C++ Usage](#c)
- [Python Usage](#python)
- [MATLAB Usage](#matlab)

## Compilation:
The package can be built either with Catkin or pure CMake. The build has been tested on Ubuntu 18.04.

## Dependencies:
- YAML-cpp
- Boost
- [erl_utilities](https://bitbucket.org/ExistentialRobotics/erl_utilities)
- [erl_env](https://bitbucket.org/ExistentialRobotics/erl_env)

### Catkin Build Instructions (Default)
Prerequisite: An existing catkin workspace containing the erl_utilities and erl_env package.

    cd ~/catkin_ws/src
    git clone git@bitbucket.org:ExistentialRobotics/erl_env.git
    catkin build

### CMake Build Instructions
From the project root directory:

    mkdir build
    cd build
    cmake .. -DUSE_ROS=OFF -DBUILD_TESTS=ON -DBUILD_PYTHON=ON
    make -j 8

## Running the tests

    mkdir build
    cd build
    cmake .. -DBUILD_TESTS=ON
    make test

Successful execution of the tests looks like:
  
      Running tests...
      Test project /home/user/workspace/erl_astar_demo/src/erl_astar/build
      Start 1: test_astar_2D
      1/8 Test #1: test_astar_2D ....................   Passed    0.12 sec
      Start 2: test_astar_cells_2D
      2/8 Test #2: test_astar_cells_2D ..............   Passed    0.04 sec
      Start 3: test_astar_2D_turbo
      3/8 Test #3: test_astar_2D_turbo ..............   Passed    0.05 sec
      Start 4: test_arastar_cells_2D
      4/8 Test #4: test_arastar_2D ..................   Passed    0.08 sec
      Start 5: test_astar_SE2
      5/8 Test #5: test_astar_SE2 ...................   Passed    0.41 sec
      Start 6: test_astar_SE2_circ
      6/8 Test #6: test_astar_SE2_circ ..............   Passed    0.03 sec
      Start 7: test_astar_3D
      7/8 Test #7: test_astar_3D ....................   Passed    0.48 sec
      Start 8: test_astar_2d_ltl
      8/8 Test #8: test_astar_2d_ltl ................   Passed    0.03 sec


## C++

From the package root:

  + `build/erl_astar/test_astar_2D data/maps/large_map_2d.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/large_map_2d.yaml ./test_path.yaml`
    
  + `build/erl_astar/test_astar_cells_2D data/maps/circles_map_2d_cell.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/circles_map_2d_cell.yaml ./test_path.yaml`

  + `build/erl_astar/test_astar_2D_turbo data/maps/large_map_2d.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/large_map_2d.yaml ./test_path.yaml`
  
  + `build/erl_astar/test_arastar_2D data/maps/large_map_2d.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/large_map_2d.yaml ./test_path.yaml`
  
  + `build/erl_astar/test_astar_SE2 data/maps/large_map_se2.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/large_map_se2.yaml ./test_path.yaml`
  
  + `build/erl_astar/test_astar_SE2 data/maps/circles_map_se2.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/circles_map_se2.yaml ./test_path.yaml`
      
  + `build/erl_astar/test_astar_3D data/maps/sikang_map_3d_cell.yaml`
  
  + `build/erl_astar/test_astar_2d_ltl data/maps/ltl/mapjie_2d.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/ltl/mapjie_2d.yaml ./test_path.yaml`

  + `build/erl_astar/test_astar_fas2d4o data/maps/large_map_2d.yaml`
  + `python ./examples/python/plot_astar_results.py ./data/maps/large_map_2d.yaml ./test_path.yaml`
  
  + `build/erl_astar/test_MotionPrimitive data/mprim/mprim_SE2_num20_knots5_dist10.yaml`

  + `build/erl_astar/test_hashing`
  
  
## Python

If you have pybind11 installed, the package will build python bindings and enable the A* planner to be called directly from Python. The following scripts demonstrate this capability, and visualize the results.

### 2D Plot
To run the A* planner:

	python3 script/astar_2d.py data/maps/large_map_2d.yaml

![](data/img/astar_2d.png)

### FAS Plot
To run the planner for the Fully Actuated System (FAS):

    python script/astar_fas.py data/maps/large_map_2d.yaml

![](data/img/astar_fas.png)
    
### SE2 Plot
To run the SE(2) planner:

    python script/astar_se2.py data/maps/large_map_se2.yaml
    
![](data/img/astar_se2.png)

### 3D Plot

Dependency: Mayavi

    pip install mayavi

To run the A* planner:

	python script/astar_3d.py data/maps/sikang_map_3d_cell.yaml

![](data/img/astar_3d.png)


## MATLAB 
### CMake (Preferred)
    cd build
    cmake .. -DBUILD_MATLAB=ON
    make -j 4
### From Command Line:
From package root:
    `mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I./include -outdir ./lib ./src/planners/jps_2D_mex.cpp ./src/planners/jps_2D.cpp` 
    `mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I./include -outdir ./lib ./src/planners/jps_3D_mex.cpp ./src/planners/jps_3D.cpp`
    `mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I./include -outdir ./lib ./src/inflate_map_mex.cpp`
    `mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I./include -I../erl_utilities/include -I/usr/include/eigen3 -lboost_system -outdir ./lib ./src/test/test_astar_2D_mex.cpp ./src/planners/astar_nx.cpp -output astar_2D_mex`
    `mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I./include -I../erl_utilities/include -I/usr/include/eigen3 -lboost_system -outdir ./lib ./src/test/test_astar_SE2_mex.cpp ./src/planners/astar_nx.cpp -output astar_SE2_mex`
    `mex CXXFLAGS='\$CXXFLAGS -std=c++11' -I./include -I../erl_utilities/include -I/usr/include/eigen3 -lboost_system -outdir ./lib ./src/planners/astar_ltl_mex.cpp ./src/planners/astar_nx.cpp` 

## Example MEX Usage:
    cd script/matlab
    matlab -nodesktop -nosplash -r "test_jps;test_astar_2d_nx;test_astar_SE2_nx"
    
Example MEX Output:

    Starting...
    Elapsed time is 0.434509 seconds.
    my A* cost = 56.445689
    Elapsed time is 0.122975 seconds.
    fast A* cost = 1128.913780



## TODO:
 
 + Fix compilation warnings
 + Clean Up MEX bindings (Low Priority)


