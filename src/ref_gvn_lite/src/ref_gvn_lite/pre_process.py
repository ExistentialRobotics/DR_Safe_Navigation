#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64
from tf.transformations import euler_from_quaternion
import os
import csv
import sys


def path_simplify(planning_path, debug=False):
    """
    Convert path message to 2d numpy array (num_pts, 2). During this process,
    simplified path, merging segments within line. for example, if 10 waypoints
    (pt1, pt2, pt3, ...pt10) are within in a stright line, convert them to (pt1, pt10)
    Note that a path can contain mutiple lines. This compression is loseless.
    """

    # check if path has more than 2 poses
    path = planning_path
    nav_path = []

    if len(path) < 2:
        rospy.logwarn_throttle(5.0, "[Preprocess Warning, path has less than 2 poses]")
        return nav_path

    # initialize two pointers for scanning the whole path
    max_idx = len(path) - 1
    nav_path = [path[0]]
    s1 = 0
    s2 = 1
    last_angle = get_theta_from_two_pts(path[0], path[1])

    while s2 <= max_idx:
        seg_start = path[s1]
        seg_end = path[s2]
        angle = get_theta_from_two_pts(seg_start, seg_end)

        if np.abs(angle - last_angle) < 1e-3:
            s2 += 1
        else:
            if debug:
                print("angle jump %.2f to %.2f add node %d" % (last_angle, angle, s2 - 1), end='')
            nav_path.append(path[s2 - 1])
            last_angle = get_theta_from_two_pts(path[s2 - 1], path[s2])  # be cautious about last angle update
            s1 = s2
            s2 += 1
            if debug:
                print("\t\tscan %d -> %d next" % (s1, s2))

    # add last node
    nav_path.append(seg_end)
    # rospy.logdebug_throttle(0.5, "Simplified path from %d points to %d points" % (len(planning_path), len(nav_path)))
    nav_path = np.round(np.array(nav_path), 2)
    return nav_path


def get_theta_from_two_pts(ptA, ptB, debug=False):
    dy = ptB[1] - ptA[1]
    dx = ptB[0] - ptA[0]
    theta = np.arctan2(dy, dx)
    if debug:
        print(ptA)
        print(ptB)
        print("[dy, dx, theta] = [%.2f, %.2f, %.2f]" % (dy, dx, theta))
    return theta


class RefGvnPreprocess:
    """ 
    Reference Governor Preprocess Module. Responsible for:
        1) taken ros 2d path from A* and simplied it (merge straight line) into numpy array path r (num_pts, 2)
        2) optional state transformation, e.g., nonlinear->linear, cartesian-> polar 
        3) taken ros odom message generate robot states z (e.g., unicycle states z = (x, y, theta))
    """

    # Running status table, higher number better status

    def __init__(self):

        # params from launch file
        _radius_topic = rospy.get_param('~radius_topic')
        _path_topic = rospy.get_param('~path_topic')
        _odom_topic = rospy.get_param('~odom_topic')

        # TODOZRD
        self._publish_simple_path = rospy.get_param('~display_nav_path')

        rospy.logwarn_once('[ref_gvn_pre] radius_topic: [%s]' % _radius_topic)

        # subscribers
        self._path_sub = rospy.Subscriber(_path_topic, Path, self.path_callback, queue_size=1)
        self._odom_sub = rospy.Subscriber(_odom_topic, Odometry, self.odom_callback, queue_size=1)
        self._radius_sub = rospy.Subscriber(_radius_topic, Float64, self.radius_callback, queue_size=1)
        self._gvn_restart_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.button_callback)
        self._gvn_located_on_unknown_status_sub = rospy.Subscriber('/gvn_located_on_unknown_status', Bool, self.gvn_located_on_unknown_status_callback)

        # publishers for display simple path (navigation path r)
        if self._publish_simple_path:
            self._nav_path_pub = rospy.Publisher("~nav_path", Path, queue_size=1)

        # upstream status variables
        self._upstream_connection = 0
        self._upstream_connection_ready = False
        self._upstream_data_ready = False
        self.gvn_restart_received = False
        self.gvn_located_on_unknown_flag = False
        self.localization_fail = False
        self.path_updated = False

        # ------------------- upstream data numpy container --------------------

        # containers for converted message in numpy
        self.path_cache = []
        self._np_path = None
        self._np_radius = None
        self._np_z = None
        self._prev_rbt_loc = None

        # cached previous robot location
        self._radius_msg = None

        self.path_received_time = rospy.Time.now()
        self.gvn_restart_received_time = rospy.Time.now()

        # ------------------- Init Upstream --------------------
        self.init_upstream()
        rospy.loginfo("[Ref Gvn Preprocessor Created!]  \n")

    def _check_upstream_connections(self, upstream_connection=3):
        """ check whether subscribers' uplink connections are established """

        self._upstream_connection = \
            self._path_sub.get_num_connections() + \
            self._odom_sub.get_num_connections() + \
            self._radius_sub.get_num_connections()

        if self._upstream_connection < upstream_connection:
            # we need to wait path, states and lidar all ready
            rospy.logwarn_throttle(1.0, '[ref_gvn_pre] waiting upstream connections [%d / %d]:', self._upstream_connection, upstream_connection)

            # path
            if self._path_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting path...")

            # odom
            if self._odom_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting odom...")

            # dist
            if self._radius_sub.get_num_connections() < 1:
                rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting radius...")
        else:
            self._upstream_connection_ready = True
            rospy.loginfo("\n[ref_gvn_pre] %d upstream connections established !\n", upstream_connection)

    def path_callback(self, path_msg):
        """
        Convert path message to 2d numpy array (num_pts, 2). During this process, 
        simplified path, merging segments within line. for example, if 10 waypoints 
        (pt1, pt2, pt3, ...pt10) are within in a stright line, convert them to (pt1, pt10)
        Note that a path can contain mutiple lines. This compression is loseless.
        """

        self.path_received_time = rospy.Time.now()
        path_dim = 2
        path_rounding_precision = 2
        rospy.loginfo_throttle(0.5, "[ref_gvn_pre] received path")
        planning_path = np.zeros((len(path_msg.poses), path_dim))
        for idx, pose in enumerate(path_msg.poses):
            planning_path[idx, 0] = pose.pose.position.x
            planning_path[idx, 1] = pose.pose.position.y

        # assume path accuray multiple of 1 cm, round to 2 precision
        planning_path.round(path_rounding_precision)
        self._np_path = path_simplify(planning_path)
        
        if len(self._np_path) == 0:
            self._np_path = self._np_z[0:2].reshape((1, -1))

        if len(self._np_path) < 2:
            self._np_path = np.row_stack((self._np_path, self._np_path))
            print("path_appended: ", self._np_path)

        self.path_cache.append(self._np_path)

        # verify simple path in rviz
        if self._publish_simple_path:
            sp_msg = Path()
            sp_msg.header = path_msg.header
            for waypoint in self._np_path:
                # create pose from waypoints
                pose = PoseStamped()
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = 0.0
                sp_msg.poses.append(pose)

            self._nav_path_pub.publish(sp_msg)

            self.path_updated = True
        return

    def button_callback(self, msg):
        """
        Trigger dummy planner.
        """
        self.gvn_restart_received = True
        self.gvn_restart_received_time = msg.header.stamp

    def odom_callback(self, odom_msg):
        rospy.logwarn_once("[ref_gvn_pre] Received odometry!")
        pose = odom_msg.pose.pose
        quaternion_sxyz = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quaternion_sxyz)
        self._np_z = np.array([pose.position.x, pose.position.y, yaw])
        self.check_de_localization()
        return

    def radius_callback(self, radius_msg):
        rospy.logwarn_once("[ref_gvn_pre] received radius")
        self._np_radius = np.array([radius_msg.data])
        return

    def _check_upstream_data(self):
        """ check whether upstream data container are loaded/initialized correctly"""
        # navigation path r
        status = True
        if self._np_path is None:
            status = False
            rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting nav_path init...")

        # robot state z
        if self._np_z is None:
            status = False
            rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting zvec init...")

        # distance info dQgO_sq
        if self._np_radius is None:
            status = False
            rospy.loginfo_throttle(1.0, "[ref_gvn_pre] waiting dist init...")

        if status:
            self._upstream_data_ready = True
            rospy.loginfo_once("\n[ref_gvn_pre] all %d upstream data initialized !\n" % self._upstream_connection)

        if self._np_z is not None and self._np_radius is None and self._np_path is None:
            # remind user to click 2D Nav Button to set goal to start reference governor
            rospy.logwarn_throttle(1.0, "[ref_gvn_pre] ------------ Click [Rviz 2D Nav Button] to Start ------------")

    def init_upstream(self):
        """ 
        Init upstream of ref_gvn adaptive tracker. 
            1. check upstream connection
            2. check upstream message and initialize downstream data containers
                for ref gvn core (nav_path r, robot state z, dist info dQgO_sq)
        """
        while (not self._upstream_connection_ready) and (not rospy.is_shutdown()):
            self._check_upstream_connections()
            rospy.sleep(0.1)  # avoid inquery too fast
            rospy.logdebug_throttle(1.0, "waiting upstream connections...")
        rospy.loginfo("[ref_gvn_pre] upstream [connection] is ready, check upstream [data]...")

        while (not self._upstream_data_ready) and (not rospy.is_shutdown()):
            self._check_upstream_data()
            rospy.sleep(0.1)  # avoid inquery too fast

        rospy.loginfo_throttle(1.0, "[ref_gvn_pre] upstream [data] is ready!")

        rospy.loginfo("[ref_gvn_pre] Upstream Init Done!")

    def gvn_located_on_unknown_status_callback(self, msg):
        """
        Set gvn_located_on_uknown_flag accroding upstream msg.
        """
        self.gvn_located_on_unknown_flag = msg.data

    def check_de_localization(self):
        """
        This function checks whether de-localization occurs.
        If the robot location different too much between in the direction perpendicular to the robot heading,
        then a de-localization is considered to be occurred.
        """
        current_rbt_loc = self._np_z[0:2]

        if self._prev_rbt_loc is None:
            self._prev_rbt_loc = self._np_z

        previous_rbt_loc = self._prev_rbt_loc[0:2]
        rbt_heading = self._np_z[2]
        dist_perp = np.abs((current_rbt_loc - previous_rbt_loc) @ np.array([[-np.sin(rbt_heading)],
                                                                            [np.cos(rbt_heading)]]))
        if dist_perp <= 0.1 and not self.localization_fail:
            self._prev_rbt_loc = self._np_z
        else:
            self.localization_fail = True
            self._np_z = self._prev_rbt_loc

    def save_planned_path_to_csv(self, *args):
        # Ensure the directory for the filename exists

        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, 'planned_path.npz')        # Save the path to a CSV file
        print("saving path", file_path)
        np.savez(file_path, *self.path_cache)
        # with open(file_path, 'w', newline='') as file:
        #     writer = csv.writer(file, quoting=csv.QUOTE_ALL)
        #     writer.writerow(self.path_cache)

        sys.exit(0)
