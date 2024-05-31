#!/usr/bin/env python3

import numpy as np
import rospy

from erl_msgs.msg import MovObsInfo, MovObsInfoArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

from erl_moving_obstacle.ros_marker_utils import create_pose_arrow_marker, create_cylinder2d_marker
from erl_moving_obstacle.virtual_movobs import MovObsStates
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D
from gazebo_msgs.msg import ModelStates


def odom_to_pose2d(msg_odom):
    """ Extract (x, y, theta) from odometry msg and converted it to pose2d.
    """
    return pose_to_pose2d(msg_odom.pose.pose)


def pose_to_pose2d(pose):
    """ Create pose2d (x, y, theta) msg from pose msg.
    """
    quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
    pose2d = Pose2D()
    pose2d.x = pose.position.x
    pose2d.y = pose.position.y
    pose2d.theta = yaw
    return pose2d


# noinspection PyShadowingNames
class MovObsWrapper:

    def __init__(self, virtual=False, virtual_list=None):
        """
        This node 
        subscribes:
            odom from (moving object detection)
            (when running "nonVirtual" mode)
        publishs:
            moving obstacle info [2d-position, 2d-velocity, radius]=[pi, vi, ri]
        """
        if virtual_list is None:
            virtual_list = []
        self.mo_array_msg = MovObsInfoArray()
        self.mo1_msg = MovObsInfo()
        self.mo2_msg = MovObsInfo()
        self.mo_array = [self.mo1_msg, self.mo2_msg]
        self.mo1_ready = False
        self.mo2_ready = False

        if not virtual:
            # params of subscribed topics from launch file
            mo1_subject_name = rospy.get_param("~mo1_subject_name")
            mo2_subject_name = rospy.get_param("~mo2_subject_name")
            # generate Vicon odom topic name
            mo1_odom_topic = "/vicon/" + mo1_subject_name + "/odom"
            mo2_odom_topic = "/vicon/" + mo2_subject_name + "/odom"
            # subscribers
            self.mo1_odom_sub = rospy.Subscriber(mo1_odom_topic, Odometry, self.mo1_odom_callback)
            self.mo2_odom_sub = rospy.Subscriber(mo2_odom_topic, Odometry, self.mo2_odom_callback)
            print(mo1_odom_topic, mo2_odom_topic)
        else:
            self.virtual_odom(virtual_list)

        # publishers
        self.marker_pub = rospy.Publisher("/mov_obs_markers", MarkerArray, queue_size=1)
        self.moinfo_pub = rospy.Publisher("/mov_obs_info_array", MovObsInfoArray, queue_size=1)

        # params of moving obstacle radius form launch file
        self.r_mo1 = rospy.get_param("~mo1_radius")
        self.r_mo2 = rospy.get_param("~mo2_radius")
        self.mo1_msg.r_mo = self.r_mo1
        self.mo2_msg.r_mo = self.r_mo2

        self.virtual = virtual
        pass

    def mo1_odom_callback(self, mo1_msg_odom):
        mo1_pose = odom_to_pose2d(mo1_msg_odom)
        self.mo1_msg.p_mo.x = mo1_pose.x
        self.mo1_msg.p_mo.y = mo1_pose.y
        self.mo1_msg.p_mo.theta = mo1_pose.theta
        self.pose_mo1 = np.array([mo1_pose.x, mo1_pose.y, mo1_pose.theta])

        mo1_twist = mo1_msg_odom.twist.twist
        self.mo1_msg.v_mo.x = mo1_twist.linear.x
        self.mo1_msg.v_mo.y = mo1_twist.linear.y

        self.mo1_ready = True
        rospy.logwarn_once("[moving obstacle 1 info] Received odometry!")
        pass

    def mo2_odom_callback(self, mo2_msg_odom):
        mo2_pose = odom_to_pose2d(mo2_msg_odom)
        self.mo2_msg.p_mo.x = mo2_pose.x
        self.mo2_msg.p_mo.y = mo2_pose.y
        self.mo2_msg.p_mo.theta = mo2_pose.theta
        self.pose_mo2 = np.array([mo2_pose.x, mo2_pose.y, mo2_pose.theta])

        mo2_twist = mo2_msg_odom.twist.twist
        self.mo2_msg.v_mo.x = mo2_twist.linear.x
        self.mo2_msg.v_mo.y = mo2_twist.linear.y

        self.mo2_ready = True
        rospy.logwarn_once("[moving obstacle 2 info] Received odometry!")
        pass

    def virtual_odom(self, mo_state_list):
        """
        generate MovObsInfo msgs from virtual moving obstacle info list for publishers
        """
        mo1_state, mo2_state = mo_state_list[0], mo_state_list[1]

        self.mo1_msg.p_mo.x = mo1_state[0]
        self.mo1_msg.p_mo.y = mo1_state[1]
        self.mo1_msg.p_mo.theta = mo1_state[2]
        self.mo1_msg.v_mo.x = mo1_state[3]
        self.mo1_msg.v_mo.y = mo1_state[4]

        self.mo2_msg.p_mo.x = mo2_state[0]
        self.mo2_msg.p_mo.y = mo2_state[1]
        self.mo2_msg.p_mo.theta = mo2_state[2]
        self.mo2_msg.v_mo.x = mo2_state[3]
        self.mo2_msg.v_mo.y = mo2_state[4]

        self.pose_mo1 = mo1_state[0:3]
        self.pose_mo2 = mo2_state[0:3]

        self.mo1_ready = True
        self.mo2_ready = True
        rospy.logwarn_once("[virtual moving obstacle] Running!")

        pass

    def publish_mo_info(self, mo_state_list=None):
        """
        publish moving obstacle info
        """
        if mo_state_list is None:
            mo_state_list = []
        if self.virtual:
            self.virtual_odom(mo_state_list)

        self.mo_array_msg.movobsinfos = self.mo_array
        self.moinfo_pub.publish(self.mo_array_msg)
        return

    def init_marker(self, arrow_dl=0.5):
        """
        Initial markers to display moving obstacle set and pose in Rviz
        """
        self.arrow_l1 = self.r_mo1 + arrow_dl
        self.arrow_l2 = self.r_mo2 + arrow_dl
        p_mo1, p_mo2 = self.pose_mo1[0:2], self.pose_mo2[0:2]
        theta_mo1, theta_mo2 = self.pose_mo1[2], self.pose_mo2[2]
        pg_mo1 = p_mo1 + np.array([self.arrow_l1 * np.cos(theta_mo1), self.arrow_l1 * np.sin(theta_mo1)])
        pg_mo2 = p_mo2 + np.array([self.arrow_l2 * np.cos(theta_mo2), self.arrow_l2 * np.sin(theta_mo2)])
        r_mo1, r_mo2 = self.r_mo1, self.r_mo2

        self.mk_mo1set = create_cylinder2d_marker(id=121, disp_h=0.1, loc2d=p_mo1,
                                                  c='dimgray', alpha=0.8, r=r_mo1, ns='mo1_set')
        self.mk_mo1pose = create_pose_arrow_marker(id=122, disp_h=0.2, start2d=p_mo1, goal2d=pg_mo1, c='dimgray')

        self.mk_mo2set = create_cylinder2d_marker(id=131, disp_h=0.1, loc2d=p_mo2,
                                                  c='dimgray', alpha=0.8, r=r_mo2, ns='mo2_set')
        self.mk_mo2pose = create_pose_arrow_marker(id=132, disp_h=0.2, start2d=p_mo2, goal2d=pg_mo2, c='dimgray')

        self.mo_markers = [self.mk_mo1set, self.mk_mo1pose, self.mk_mo2set, self.mk_mo2pose]
        self.marker_pub.publish(self.mo_markers)
        return

    def update_marker(self):
        """
        Update non-static markers in Rviz
        """
        p_mo1, p_mo2 = self.pose_mo1[0:2], self.pose_mo2[0:2]
        theta_mo1, theta_mo2 = self.pose_mo1[2], self.pose_mo2[2]
        pg_mo1 = p_mo1 + np.array([self.arrow_l1 * np.cos(theta_mo1), self.arrow_l1 * np.sin(theta_mo1)])
        pg_mo2 = p_mo2 + np.array([self.arrow_l2 * np.cos(theta_mo2), self.arrow_l2 * np.sin(theta_mo2)])

        self.mk_mo1set.pose.position.x = p_mo1[0]
        self.mk_mo1set.pose.position.y = p_mo1[1]
        self.mk_mo1pose.points[0] = Point(p_mo1[0], p_mo1[1], 0.0)
        self.mk_mo1pose.points[1] = Point(pg_mo1[0], pg_mo1[1], 0.0)

        self.mk_mo2set.pose.position.x = p_mo2[0]
        self.mk_mo2set.pose.position.y = p_mo2[1]
        self.mk_mo2pose.points[0] = Point(p_mo2[0], p_mo2[1], 0.0)
        self.mk_mo2pose.points[1] = Point(pg_mo2[0], pg_mo2[1], 0.0)

        self.marker_pub.publish(self.mo_markers)
        return


class GazeboMovingObstacle:

    def __init__(self, virtual_list=None):
        """
        This node
        subscribes:
            odom from (moving object detection)
        publishs:
            moving obstacle info [2d-position, 2d-velocity, radius]=[pi, vi, ri]
        """

        # Init moving obstacle parameters
        if virtual_list is None:
            self.virtual_list = ['actor1', 'actor2', 'actor4']
        else:
            self.virtual_list = virtual_list
        self.size_virtual = len(self.virtual_list)

        # Init container
        self.mo_array = [MovObsInfo() for i in range(self.size_virtual)]
        self.mo_array_msg = MovObsInfoArray()
        self.mo_markers = []

        # Status variable
        self.mov_ready = False

        # Record time
        self.prev_time = rospy.Time.now()

        # Receive parameters
        moving_obs_radius = rospy.get_param("~mov_obs_radius", [0.5 for i in range(self.size_virtual)])

        for i in range(self.size_virtual):
            self.mo_array[i].r_mo = moving_obs_radius[i]

        # subscribers
        self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback)

        # publishers
        self.marker_pub = rospy.Publisher("/mov_obs_markers", MarkerArray, queue_size=1)
        self.moinfo_pub = rospy.Publisher("/mov_obs_info_array", MovObsInfoArray, queue_size=1)

    def gazebo_callback(self, gazebo_msg):
        """
        Obtain moving obstacle position from Gazebo, velocity manually calculated,
        Gazebo is publishing model states at a really high frequency, such that the time difference
        between to messages often decreases to zero. Only approximate velocity when time
        difference is non-zero.
        """
        now_time = rospy.Time.now()
        delta_time = (now_time - self.prev_time).to_sec()
        self.prev_time = now_time
        for i in range(self.size_virtual):
            ind = gazebo_msg.name.index(self.virtual_list[i])
            pose = pose_to_pose2d(gazebo_msg.pose[ind])
            if self.mov_ready:
                if delta_time != 0:
                    self.mo_array[i].v_mo.x = (pose.x - self.mo_array[i].p_mo.x) / delta_time
                    self.mo_array[i].v_mo.y = (pose.y - self.mo_array[i].p_mo.y) / delta_time
                    self.mo_array[i].v_mo.z = (pose.theta - self.mo_array[i].p_mo.theta) / delta_time

            self.mo_array[i].p_mo.x = pose.x
            self.mo_array[i].p_mo.y = pose.y
            self.mo_array[i].p_mo.theta = pose.theta - np.pi/2

            self.mov_ready = True
            rospy.logwarn_once("[moving obstacles] Received odometry!")

    def publish_mov_obs_info(self):
        """
        Publish moving obstacle information
        """
        if self.mov_ready:
            self.mo_array_msg.movobsinfos = self.mo_array
            self.moinfo_pub.publish(self.mo_array_msg)

    def init_marker(self, arrow_dl=0.5):
        """
        Initial markers to display moving obstacle set and pose in Rviz
        """
        for i in range(self.size_virtual):
            id_set = 100 + 10 * i + 1
            id_arr = 100 + 10 * i + 2
            position = np.array([self.mo_array[i].p_mo.x, self.mo_array[i].p_mo.y])
            theta = self.mo_array[i].p_mo.theta
            radius = self.mo_array[i].r_mo
            name = "mo" + str(i) + "set"
            arrow = radius + arrow_dl
            point_goal = position + np.array([arrow * np.cos(theta), arrow * np.sin(theta)])
            mk_set = create_cylinder2d_marker(id=id_set, disp_h=0.1, loc2d=position,
                                              c='dimgray', alpha=0.8, r=radius, ns=name)
            self.mo_markers.append(mk_set)
            mk_pose = create_pose_arrow_marker(id=id_arr, disp_h=0.2, start2d=position,
                                               goal2d=point_goal, c='dimgray')
            self.mo_markers.append(mk_pose)

        self.marker_pub.publish(self.mo_markers)

    def update_marker(self, arrow_dl=0.5):
        """
        Update non-static markers in Rviz
        """
        for i in range(self.size_virtual):
            position = np.array([self.mo_array[i].p_mo.x, self.mo_array[i].p_mo.y])
            theta = self.mo_array[i].p_mo.theta
            radius = self.mo_array[i].r_mo
            arrow = radius + arrow_dl
            point_goal = position + np.array([arrow * np.cos(theta), arrow * np.sin(theta)])

            self.mo_markers[2*i].pose.position.x = position[0]
            self.mo_markers[2*i].pose.position.y = position[1]

            self.mo_markers[2*i + 1].points[0] = Point(position[0], position[1], 0.0)
            self.mo_markers[2*i + 1].points[1] = Point(point_goal[0], point_goal[1], 0.0)

        self.marker_pub.publish(self.mo_markers)


if __name__ == '__main__':

    try:
        rospy.init_node("mov_obs_info")
        rospy.logwarn("bring up [mov_obs_info] node!\n")

        VirtualMode = rospy.get_param("~virtual", False)
        SimulationMode = rospy.get_param("~simulation", True)
        Marker_Init = False
        rate = rospy.Rate(100.0)

        if not SimulationMode:
            if VirtualMode:
                # define moving obstacle start & goal positon, linear velocity
                t_idx = 0
                dt = 0.01
                t = t_idx * dt
                # load params from launch file
                p_start_mo1_x = rospy.get_param("~mo1_pstart_x")
                p_start_mo1_y = rospy.get_param("~mo1_pstart_y")
                p_start_mo2_x = rospy.get_param("~mo2_pstart_x")
                p_start_mo2_y = rospy.get_param("~mo2_pstart_y")
                p_goal_mo1_x = rospy.get_param("~mo1_pgoal_x")
                p_goal_mo1_y = rospy.get_param("~mo1_pgoal_y")
                p_goal_mo2_x = rospy.get_param("~mo2_pgoal_x")
                p_goal_mo2_y = rospy.get_param("~mo2_pgoal_y")

                p_start_mo1 = np.array([p_start_mo1_x, p_start_mo1_y])
                p_goal_mo1 = np.array([p_goal_mo1_x, p_goal_mo1_y])
                v_mo1 = rospy.get_param("~mo1_v")

                p_start_mo2 = np.array([p_start_mo2_x, p_start_mo2_y])
                p_goal_mo2 = np.array([p_goal_mo2_x, p_goal_mo2_y])
                v_mo2 = rospy.get_param("~mo2_v")

                mo1_info = [p_start_mo1, p_goal_mo1, v_mo1]
                mo2_info = [p_start_mo2, p_goal_mo2, v_mo2]
                MO1 = MovObsStates(mo1_info)
                MO2 = MovObsStates(mo2_info)

                mo1_state = MO1.get_moving_obstacle_state(t)
                mo2_state = MO2.get_moving_obstacle_state(t)
                mo_state_list = [mo1_state, mo2_state]
                mov_obs = MovObsWrapper(VirtualMode, mo_state_list)

            else:
                mov_obs = MovObsWrapper(VirtualMode)
                mov_obs.publish_mo_info()

            while not rospy.is_shutdown():
                if mov_obs.mo1_ready and mov_obs.mo2_ready and (not Marker_Init):
                    rospy.logwarn("[mov_obs_info] ready!\n")
                    mov_obs.init_marker()
                    Marker_Init = True

                if VirtualMode:
                    # publish virtual moving obstacle states
                    t = t_idx * dt
                    mo1_state = MO1.get_moving_obstacle_state(t)
                    mo2_state = MO2.get_moving_obstacle_state(t)
                    mo_state_list = [mo1_state, mo2_state]
                    mov_obs.publish_mo_info(mo_state_list)
                    t_idx += 1
                else:
                    # print( "publish captured moving obstacle states")
                    mov_obs.publish_mo_info()

                if Marker_Init:
                    mov_obs.update_marker()
                rate.sleep()

        else:
            sim_mov_obs = GazeboMovingObstacle()

            while not rospy.is_shutdown():
                sim_mov_obs.publish_mov_obs_info()

                if sim_mov_obs.mov_ready and not Marker_Init:
                    sim_mov_obs.init_marker()
                    Marker_Init = True

                if Marker_Init:
                    sim_mov_obs.update_marker()

                rate.sleep()

    except rospy.ROSInterruptException:
        rospy.logerr("[mov_obs_info] node init failed.")
        rospy.signal_shutdown("[mov_obs_info] node init fail")
        pass
