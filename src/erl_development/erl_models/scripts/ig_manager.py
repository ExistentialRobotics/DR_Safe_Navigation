#!/usr/bin/env python

import rospy
import math
# ROS Message Types
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3, PoseWithCovarianceStamped
#from tf.transformations import quaternion_from_euler, euler_from_quaternion
#from misc import quaternion_from_euler
from nav_msgs.msg import Path
from erl_msgs.msg import PrimitiveTrajectory, Primitive

class IGManager(object):

    def robot_callback(self, odom, index):
        self.robot_states[index] = odom

    def target_callback(self, odom, index):
        self.target_states[index] = odom

    def create_pose_stamped(self, x, y, z, yaw):
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z

        # Fill in Yaw

        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw/2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw/2.0)
        return pose_stamped

    def publish_robot_wp(self, waypoint, idx, z=0.0):
        self.robot_pub_list[idx].publish(self.create_pose_stamped(waypoint[0], waypoint[1], z, waypoint[2]))
        # print('Published a waypoint for Robot_' + str(idx) + 'at', waypoint)

    def publish_quadrotor_traj(self, traj, idx):
        print('PUBLISHING QUADROTOR TRAJECTORY, ', idx)
        self.quadrotor_pub_list[idx].publish(traj)

    def publish_target_wp(self, waypoint, idx, z=0.0):
        self.target_pub_list[idx].publish(self.create_pose_stamped(waypoint[0], waypoint[1], z, waypoint[2]))

    def publish_target_belief(self, state, covariance, idx, z=0.0):
        # Create PoseWithCovariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id='map'
        # Pose
        pose_msg.pose.pose = self.create_pose_stamped(state[0], state[1], z, state[2]).pose
        # Covariance with only X, Y uncertainty given.
        pose_msg.pose.covariance[0] = covariance[0, 0]
        pose_msg.pose.covariance[1] = covariance[0, 1]
        pose_msg.pose.covariance[6] = covariance[1, 0]
        pose_msg.pose.covariance[7] = covariance[1, 1]

        # Publish the message.
        self.target_belief_pub_list[idx].publish(pose_msg)

    def publish_robot_path(self, path, idx, frame='map', z=0.0):
        # Create Path Message
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.poses = [self.create_pose_stamped(pos[0], pos[1], 0, pos[2]) for pos in path]
        self.robot_path_pub_list[idx].publish(path_msg)

    def publish_target_path(self, path, idx, frame='map', z=0.0):
        # Create Path Message
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.poses = [self.create_pose_stamped(pos[0], pos[1], 0, pos[2]) for pos in path]
        self.target_path_pub_list[idx].publish(path_msg)

    def __init__(self):
        rospy.init_node('IGManager')

        # Setup Parameters
        self.num_robots = rospy.get_param('~num_robots')
        self.num_targets = rospy.get_param('~num_targets')
        self.publish_paths = rospy.get_param('~publish_paths')
        self.data = rospy.get_param('~data')

        # Initialize Robot and Target Pose Arrays
        self.robot_states = [None] * self.num_robots
        self.target_states = [None] * self.num_targets

        # Setup Subscribers
        self.sub_list = []
        for i in range(0, self.num_robots):  # For Robots
            self.sub_list.append(
                rospy.Subscriber('robot_sub_' + str(i), Odometry, self.robot_callback, i, queue_size=10))
        for i in range(0, self.num_targets):  # For Targets
            self.sub_list.append(
                rospy.Subscriber('target_sub_' + str(i), Odometry, self.target_callback, i, queue_size=10))

        # Setup Publishers
        self.robot_pub_list = []
        self.quadrotor_pub_list = []
        for i in range(0, self.num_robots):  # For Robots
            self.robot_pub_list.append(rospy.Publisher('robot_pub_' + str(i), PoseStamped, queue_size=10))
            self.quadrotor_pub_list.append(rospy.Publisher('quadrotor_pub_'+str(i), PrimitiveTrajectory, queue_size=10))

        self.target_pub_list = []
        self.target_belief_pub_list = []
        for i in range(0, self.num_targets):  # For Targets
            self.target_pub_list.append(rospy.Publisher('target_pub_' + str(i), PoseStamped, queue_size=10))
            self.target_belief_pub_list.append(rospy.Publisher('target_belief_pub_' + str(i), PoseWithCovarianceStamped, queue_size=10))

        # Setup Path Publishers
        self.robot_path_pub_list = []
        self.target_path_pub_list = []
        if self.publish_paths:
            for i in range(0, self.num_robots):  # For Robots
                self.robot_path_pub_list.append(rospy.Publisher('robot_path_pub_' + str(i), Path, queue_size=10))
            for i in range(0, self.num_targets):  # For Targets
                self.target_path_pub_list.append(rospy.Publisher('target_path_pub_' + str(i), Path, queue_size=10))
# End Class
