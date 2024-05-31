Distributively Robust Safe Robot Navigation
===========================================

[[Website]](https://existentialrobotics.org/DR_Safe_Navigation_Webpage/)
[[Paper]](https://arxiv.org/abs/2405.18251)

This repository contains implementations for the work "Sensor-Based Distributively Robust Control for Safe Robot Navigation in Dynamic Environments".

If you find our work useful, please consider citing our paper:
```
@article{long2024sensorbased,
  title={Sensor-Based Distributionally Robust Control for Safe Robot Navigation in Dynamic Environments}, 
  author={Kehan Long and Yinzhuang Yi and Zhirui Dai and Sylvia Herbert and Jorge Cortés and Nikolay Atanasov},
  journal={arXiv preprint arXiv:2405.18251},
  year={2024}
}
```

# 🛝 Try it out!
```bash
git clone https://github.com/ExistentialRobotics/DR_Safe_Navigation.git
```

## Dependencies
The code is tested on Ubuntu 20.04 LTS + ROS Noetic, in both simulation and real experiments. 


If you computer dose not have ROS installed, please follow the ROS installation guidelines:
```
http://wiki.ros.org/noetic/Installation/Ubuntu
```

### ROS libraries
```
sudo apt install ros-noetic-pybind11-catkin \
  python3-catkin-tools  \
  libcgal-dev  \
  liblapacke-dev \
  python3-pytest \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-tf-conversions \
  ros-noetic-rviz \
  python3-numpy \
  python3-scipy \
  libboost-all-dev \
  ros-noetic-joint-state-controller \
  ros-noetic-effort-controllers \
  ros-noetic-position-controllers \
  ros-noetic-hector-mapping
```
and the following dependencies for Jackal simulation:
```
sudo apt-get install "ros-noetic-jackal*"  &&
sudo apt install ros-noetic-map-server  &&
sudo apt install 'ros-noetic-rqt*'
```

### Non-ROS libraries
  + Eigen3  `sudo apt install -y libeigen3-dev`
  + Boost `sudo apt install-y libboost-dev`
  + OPENCV `sudo apt install libopencv-dev python3-opencv`
  + CVXPY `pip install cvxpy`
  + CGAL `sudo apt install libcgal-dev`

For the Gazebo dynamic environment simulation, additionally, you would need to install the <a href="https://github.com/robotics-upo/lightsfm?tab=readme-ov-file#lightsfm">lightsfm</a>.


## Compilation

```catkin build erl_clf_cbf_controller```
and then
```source devel/setup.bash```
to source the environment.

# 👩‍💻 Code (Gazebo Simulation)

## (Optional) Add noise to LiDAR
To test the robustness of the controller, one can modify the noise model of the lms1xx 2D LiDAR in the Gazebo simulation. To do so, first run ```roscd lms1xx```, then ```cd  urdf/```, and modify the ```sick_lms1xx.urdf.xacro``` file, there is an 'Stddev' variable to modify. (If ```roscd lms1xx``` does not work, one could ```cd /opt/ros/noetic/share/lms1xx``` to the desired directory.) 

## 🛠️ Launch

1. To run the Gazebo simulation in the static environment, use the following command:
```
roslaunch erl_clf_cbf_controller clf_cbf_control.launch
```

2. To run the Gazebo simulation in the dynamic environment, use the following command:
```
roslaunch erl_clf_cbf_controller clf_cbf_mov_obs.launch
```

If the robot is not being spawned successfully, refer to this link:
```
https://www.clearpathrobotics.com/assets/guides/kinetic/jackal/simulation.html
```

# Baseline compare (nominal CLF-CBF QP and CLF GP-CBF SOCP)

To compare the controller performance with the provided baselines in Gazebo, you could specify two options when launching the robot: controller_type and use_sdf. The default setting is controller_type:=drccp, use_sdf:=false.

There are three more controller types: clf_qp_only (no collision avoidance), baseline_clf_cbf_qp, and gp_cbf_socp. To use the GP-SDF map, set use_sdf:=true. 

For example, if you want to run the GP-SOCP controller with the GP-SDF map, use the following command:
```
roslaunch erl_clf_cbf_controller clf_cbf_control.launch controller_type:=gp_cbf_socp use_sdf:=true
```

**Note**: To use the GP-SDF map for controller synthesis. Please run
```catkin build ```
to build all the required packages (you will need to install additional packages as below).  


## Install Additional Packages (For GP-SDF Mapping)
If you want to use the GP-SDF map for the controller synthesis, you would need to install the following additional packages. 

Install [intel-basekit](https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit-download.html?operatingsystem=linux&distributions=aptpackagemanager):
```
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \ | gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | sudo tee /etc/apt/sources.list.d/oneAPI.list
sudo apt update
sudo apt install intel-basekit
```

Install Eigen 3.4.90
```
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
mkdir build
cd build
cmake ..
sudo cmake --build . --target install -j $(nproc)
cd ../..
rm -rf eigen
```

Install yaml-cpp 0.7.0
```
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
git checkout 0e6e28d
mkdir build
cd build
cmake .. -DYAML_BUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
sudo cmake --build . --target install -j$(nproc)
cd ../..
rm -rf yaml-cpp
```

Install abseil-cpp
```
wget https://github.com/abseil/abseil-cpp/releases/download/20240116.1/abseil-cpp-20240116.1.tar.gz
tar -xf abseil-cpp-20240116.1.tar.gz
cd abseil-cpp-20240116.1
mkdir build
cd build
cmake .. cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_FLAGS="${CXXFLAGS} -DNDEBUG" -DCMAKE_CXX_STANDARD=17 -DBUILD_SHARED_LIBS=ON -DABSL_PROPAGATE_CXX_STD=ON
make -j`nproc`
sudo make install
```

**Note**:
If `/usr/local/lib` is not in `LD_LIBRARY_PATH`, you should modify your environment setup to make sure abseil-cpp libraries are loaded correctly:
```bash
export LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
```

Install fmt 9.1.0
```bash
wget https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.tar.gz
tar -xf 9.1.0.tar.gz
mkdir -p fmt-9.1.0/build && cd fmt-9.1.0/build
cmake .. -DFMT_DOC=OFF -DFMT_TEST=OFF -DFMT_INSTALL=ON -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON 
make -j`nproc` 
sudo make install
```

## Alternative: Use Docker
A script is provided to build the docker image for the project. To use the script, you need to install docker and nvidia-docker first.
```bash
cd docker
chmod +x build.bash
./build.bash
```

# 🏷️ License
This repository is released under the MIT license. 


