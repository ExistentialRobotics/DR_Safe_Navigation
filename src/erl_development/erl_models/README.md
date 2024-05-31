# ERL Models: A package containing world, object, and robot models and Launch Files.


## Table of Contents:

- [Dependencies](#dependencies)
- [Description](#description)
- [Launch Files](#launch-files)
	- [Launching a Simulation](#launching-a-simulation)
	- [Creating new Launch Files](#creating-new-launch-files)
		- [Spawning the World](#spawning-the-world)
		- [Spawning a Robot](#spawning-a-robot)
		- [Localization and Mapping](#localization-and-mapping)
		- [Navigation](#navigation)

## Dependencies
1. ROS Kinetic / Melodic [Instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)

2. Catkin Simple
	
		git clone https://github.com/catkin/catkin_simple.git

3. GMapping

		git clone https://github.com/ros-perception/openslam_gmapping.git
		git clone https://github.com/ros-perception/slam_gmapping.git

## Description

This package provides URDF and mesh models for various robots and worlds. These models may be utilized by other modules in the stack. For an in-depth guide on the model types, see:
- URDF: (http://wiki.ros.org/urdf)
- Meshes: (TODO)
- Worlds: (TODO)

The following robot models are currently supported:

- Quadrotor (hummingbird, with or without RGBD)
- Scarab (with kinect, VI_sensor, Velodyne)
- Mobile Hokuyo (floating Hokuyo sensor)
- Mobile RGBD (floating RGBD sensor)
- Mobile Velodyne (floating Velodyne sensor)

The following world models are currently supported:

- fla_warehouse1.world
- levine.world
- empty.world

For examples of how to launch simulations with these models, please see the [erl_models](https://bitbucket.org/natanaso/information_acquisition/src/master/packages/erl_models/) package.


## Launch Files
This package contains the launch files for simulating various robot and sensor configurations, through ROS and Gazebo. The environments are displayed in RVIZ. 


### Launching a Simulation
For instance, to launch two mobile_hokuyo robots (i.e. floating hokuyo laser sensors):

```roslaunch erl_launch mobile_hokuyo.launch```

Other examples:

	roslaunch erl_models scarab.launch
	roslaunch erl_models pelican_laser.launch
	roslaunch erl_models ground_air.launch
	roslaunch erl_models mobile_hokuyo.launch


### Creating new Launch Files
We examine a complete launch file to demonstrate how one can write new or customized launch files.

```xml
<?xml version="1.0" encoding="ISO-8859-15"?>
<launch>
  <arg name="ns" default="scarab40" />
  
	<!-- Spawn the World -->
  <include file="$(find erl_models)/launch/spawn_world.launch">
    <arg name="world" value="fla_warehouse1"/>
    <arg name="scale" value="0.0254"/>
    <arg name="gui" value="false"/>
  </include>

  <!-- Spawn the Scarab -->
  <include file="$(find erl_models)/launch/spawn_robot.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    <arg name="model" value="$(find erl_models)/urdf/scarab/scarab.xacro"/>
    <arg name="px" value="5.0" />
    <arg name="py" value="0.0" />
    <arg name="use_groundtruth_odom" value="false" />
  </include>

  <!-- Run Scarab Localization -->
  <include file="$(find erl_models)/launch/scarab/scarab_localization.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    <arg name="base_frame" value="base_footprint"/>
    <arg name="laser_frame" value="laser"/>
    <arg name="odom_laser_frame" value="odom"/>
    <arg name="scan_topic" value="scan" />
    <arg name="odom_motor_topic" value="odom" />
    <arg name="odom_laser_topic" value="odom_laser" />
  </include>
    
  <!-- Run Scarab Mapping -->
  <include file="$(find erl_models)/launch/scarab/scarab_mapping.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    <arg name="scan_topic" value="scan" />
    <arg name="base_frame" value="base_footprint"/>
    <arg name="odom_frame" value="odom"/>
    <arg name="map_frame"  value="map"/>        
  </include>
  
  <!-- Run Scarab Navigation -->
  <include file="$(find erl_models)/launch/scarab/scarab_navigation.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    
    <arg name="map_frame"  value="map"/>
    <arg name="odom_frame" value="odom"/>
    <arg name="base_frame" value="base_footprint"/>
    
    <arg name="map_topic"  value="map"/>
    <arg name="odom_topic" value="odom"/>
    <arg name="scan_topic" value="scan" />
    <arg name="pose_topic" value="pose" />
    <arg name="goal_topic" value="move_base_simple/goal"/>
    
    <arg name="use_hfn" value="true"/>   
  </include>  

  <!-- Launch rviz -->
  <node pkg="rviz" type="rviz" name="rviz"
        args="-d $(find erl_models)/rviz/scarab40.cfg.rviz"/>
  

</launch>
```

#### Spawning the World:

```xml
	<!-- Spawn the World -->
  <include file="$(find erl_models)/launch/spawn_world.launch">
    <arg name="world" value="fla_warehouse1"/>
    <arg name="scale" value="0.0254"/>
    <arg name="gui" value="false"/>
  </include>
```
The file ```spawn_world.launch``` launches a gazebo simulation with the specified world. This passes the appropriate world (meshes) to both Gazebo and RViz for simulation and visualization respectively. The scale parameter should be set to 1 for ROS Kinetic, and .0254 for ROS Melodic due to issues with the world mesh files. The GUI parameter indicates whether the Gazebo GUI will spawn or not (we set it to false here because we visualize things in RVIZ).

#### Spawning a Robot:
```xml
  <!-- Spawn the Scarab -->
  <include file="$(find erl_models)/launch/spawn_robot.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    <arg name="model" value="$(find erl_models)/urdf/scarab/scarab.xacro"/>
    <arg name="px" value="5.0" />
    <arg name="py" value="0.0" />
    <arg name="use_groundtruth_odom" value="false" />
  </include>
```

The file ```spawn_robot.launch``` spawns a robot in ROS/Gazebo. This file must be given a namespace and tf_prefix (these are required when multiple robots may be spawned in the same environment), a path to a URDF model, coordinates relative to the world origin for the initial spawn location. Optionally, this file can be given the flag `use_groundtruth_odom`, which spawns a publisher on `robot_ns/odom` that publishes the ground truth robot odometry. This is useful for simulating algorithms where the localization is known. See the [erl_models](https://bitbucket.org/natanaso/information_acquisition/src/master/packages/erl_models/) package for the currently supported robot models.

#### Localization and Mapping:
```xml
  <!-- Run Scarab Localization -->
  <include file="$(find erl_models)/launch/scarab/scarab_localization.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    <arg name="base_frame" value="base_footprint"/>
    <arg name="laser_frame" value="laser"/>
    <arg name="odom_laser_frame" value="odom"/>
    <arg name="scan_topic" value="scan" />
    <arg name="odom_motor_topic" value="odom" />
    <arg name="odom_laser_topic" value="odom_laser" />
  </include>
    
  <!-- Run Scarab Mapping -->
  <include file="$(find erl_models)/launch/scarab/scarab_mapping.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    <arg name="scan_topic" value="scan" />
    <arg name="base_frame" value="base_footprint"/>
    <arg name="odom_frame" value="odom"/>
    <arg name="map_frame"  value="map"/>        
  </include>
```

The files here spawn localization and mapping nodes. See the documentation for [openslam_gmapping](https://github.com/ros-perception/openslam_gmapping) for the configuration of these.

#### Navigation:
```xml
  <!-- Run Scarab Navigation -->
  <include file="$(find erl_models)/launch/scarab/scarab_navigation.launch">
    <arg name="ns" value="$(arg ns)"/>
    <arg name="tf_prefix" value="$(arg ns)" />
    
    <arg name="map_frame"  value="map"/>
    <arg name="odom_frame" value="odom"/>
    <arg name="base_frame" value="base_footprint"/>
    
    <arg name="map_topic"  value="map"/>
    <arg name="odom_topic" value="odom"/>
    <arg name="scan_topic" value="scan" />
    <arg name="pose_topic" value="pose" />
    <arg name="goal_topic" value="move_base_simple/goal"/>
    
    <arg name="use_hfn" value="true"/>   
  </include>  
```

The file here spawns a scarab navigation node using HFN (Human Friendly Navigation). For other navigation nodes, see the documentation for HFN, or for example the [erl_astar_ros](https://bitbucket.org/ExistentialRobotics/erl_astar_ros/src/master/) package.

<!-- 
The simulation launcher file calls two general files. The first is ```spawn_world.launch```, which must be given the ```world``` argument. This passes the appropriate world (meshes) to both Gazebo and RViz for simulation and visualization respectively.

The second file is the ```spawn_robot.launch``` file. This must be given a namespace (this is useful when multiple robots may be spawned in the same environment), a path to a URDF model, coordinates relative to the world origin, and optionally a mesh to draw the robot in RVIZ. For more information on the models, see the [erl_models](https://bitbucket.org/natanaso/information_acquisition/src/master/packages/erl_models/) package.
 -->


## Known Issues:

 + To fix the error: "libpng warning: iCCP: known incorrect sRGB profile"
    ```sh
    cd /opt/ros/melodic/share/rviz
    mogrify *.png in all subdirectories
    ```
+ Spawn service failure. This error message appears on ROS melodic, but does not seem to cause any further issues. There is no known solution.

		[ERROR] [1546876879.495905, 1812.387000]: Spawn service failed. Exiting.
		[scarab40/spawn_robot-5] process has died [pid 7175, exit code 1, cmd /opt/ros/melodic/lib/gazebo_ros/spawn_model -urdf -model scarab40 -param robot_description -x 5.0 -y 0.0 -z 0.0 -Y 0.0 -R 0.0 -P 0.0 __name:=spawn_robot 

## Using RTABMAP with simulation:

1. Install rtabmap: `sudo apt-get install ros-melodic-rtabmap-ros`
2. Source the working directory
3. `roslaunch erl_models scarab_rtabmap.launch` (default: teleop with keyboard)

