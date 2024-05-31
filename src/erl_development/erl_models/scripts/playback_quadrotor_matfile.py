#!/usr/bin/env python


from ig_manager import IGManager
import scipy.io as sio
import numpy as np
import rospy
import pdb
from erl_msgs.msg import Primitive, PrimitiveTrajectory
import time

def primitive1DAcc(p1,v1,a1,p2,v2,a2,t):
  t2 = t*t
  t3 = t2*t
  t4 = t3*t
  t5 = t4*t
  c = [-(60*(12*p1 - 12*p2 +  6*t*v1 +  6*t*v2 +   a1*t2 -   a2*t2))/t5,\
        (12*(30*p1 - 30*p2 + 16*t*v1 + 14*t*v2 + 3*a1*t2 - 2*a2*t2))/t4,\
       -(3* (20*p1 - 20*p2 + 12*t*v1 +  8*t*v2 + 3*a1*t2 -   a2*t2))/t3,\
        a1,v1,p1]
  return c

def primitive1DVel(p1,v1,p2,v2,t):
  t2 = t*t
  t3 = t2*t
  c = [0.0,0.0,\
       (6*(2*p1 - 2*p2 + t*v1 + t*v2))/t3,\
      -(2*(3*p1 - 3*p2 + 2*t*v1 + t*v2))/t2,\
       v1, p1]
  return c

def primitive1DPos(p1,p2,t):
  c = [p1, (p2-p1)/t, 0.0, 0.0, 0.0, 0.0, 0.0]
  # c = [0.0, 0.0, 0.0, 0.0, (p2-p1)/t, p1]
  return c



'''Reads in a MAT file containing Robot Paths, Target Paths, and Robot beliefs and publishes to ROS.'''
if __name__ == "__main__":
    node = IGManager()
    filename = node.data

    data = sio.loadmat(filename)  # Load the desired MAT File
    results = data['results']
    GroundTruth = results['GroundTruth'][0, 0]
    # InitialBelief = results['InitialBelief'][0, 0]
    # PathOfTargets = results['NoiselessPathOfTargets'][0][0]
    PathOfTargets = results['PathOfTargets'][0][0]
    CostOfBestPath = results['CostOfBestPath']
    Horizon = results['Horizon'][0, 0][0, 0]
    RobotTheta = results['RobotTheta'][0, 0]
    RobotPositions = results['RobotPositions'][0, 0]
    ControlInputs = results['ControlInputs']  # Unused.
    Covariance = results['Cov'][0, 0]
    SensingRange = results['SensingRange'][0, 0]
    tau = results['tau'][0,0]
    # Map Offsets
    x_off = + 8.8223
    y_off = + 8.8223 - .5

    # Load Robot Paths
    num_robots =1 #  np.shape(RobotTheta)[1]
    robot_paths = [[np.array([pos[0] + x_off, pos[1] + y_off, th]) for pos, th in
                    zip(RobotPositions[:, (i * 2):(2 * i) + 2], RobotTheta[:, i])] for i in
                   range(0, num_robots)]
    
    # Generate Robot Trajectories
    z_val = 1.0
    robotTrajectories = [PrimitiveTrajectory() for _ in range(num_robots)]
    for i in range(num_robots):
      robotTrajectories[i].header.frame_id = 'map'
      robotTrajectories[i].primitives = [Primitive() for _ in range(Horizon-1)]
      for t in range(1, Horizon):
        p1 = np.append(robot_paths[i][t-1][:2],z_val)
        p2 = np.append(robot_paths[i][t][:2],z_val)
        t12 = 0.2
        robotTrajectories[i].primitives[t-1].header.frame_id = 'map'
        robotTrajectories[i].primitives[t-1].t = t12
        robotTrajectories[i].primitives[t-1].cx = primitive1DPos(p1[0],p2[0],t12)
        robotTrajectories[i].primitives[t-1].cy = primitive1DPos(p1[1],p2[1],t12)
        robotTrajectories[i].primitives[t-1].cz = primitive1DPos(p1[2],p2[2],t12)
    
    # Load Target Paths
    num_targets = 0 # np.shape(PathOfTargets.T)[1] / 2
    target_th = [np.pi / 2, - np.pi/6, np.pi, -np.pi/2, 0, 0]
    target_paths = [[np.array([pos[0] + x_off, pos[1] + y_off, target_th[i]])
                     for pos in PathOfTargets.T[:, 2 * i:2 + 2 * i]] for i in range(0, num_targets)]
    # Load Target Covariance
    target_cov = Covariance
    rate = rospy.Rate(5)

    while node.robot_states[0] is None:  # Sleep Until ODOM is received to ensure things are set up.
        rate.sleep()

    time.sleep(5)

    # Publish Robot Trajectories
    for i in range(0, num_robots):
      node.publish_quadrotor_traj(robotTrajectories[i], i)
    
    # Main loop
    for t in range(0, Horizon):

        # Publish Robot Waypoints
        #for i in range(0, num_robots):
        #    node.publish_robot_wp(robot_paths[i][t], i, z=0.0)

        # Publish Target Waypoints and Belief
        covariance_matrices = [target_cov[0, t][2*i:2*i+2, 2*i:2*i+2] for i in range(0, num_targets)]
        for i in range(0, num_targets):
            node.publish_target_wp(target_paths[i][t], i, z=0.0)
            node.publish_target_belief(target_paths[i][t], covariance_matrices[i], i, z=0.0)

        if node.publish_paths:
            for i in range(0, num_robots):             # Publish Robot Paths
                node.publish_robot_path(robot_paths[i], i, z=0.0)
                # node.publish_robot_path(robot_paths[i][0:t], i, z=0.0)
            for i in range(0, num_targets):             # Publish Target Paths
                node.publish_target_path(target_paths[i], i, z=0.0)
                # node.publish_target_path(target_paths[i][0:t], i, z=0.0)

        rate.sleep()
    # End Main Loop
