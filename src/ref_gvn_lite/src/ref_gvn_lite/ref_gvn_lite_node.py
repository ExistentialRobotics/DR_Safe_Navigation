#!/usr/bin/env python3
""" Safe path-following controller based on reference governor.
"""

from __future__ import print_function

import rospy
import numpy as np
from geometry_msgs.msg import Point, Pose2D

from pre_process import RefGvnPreprocess
from post_process import RefGvnPostprocess
from core_lite import RefGvnLiteCore
from visualization_msgs.msg import MarkerArray
from erl_msgs.msg import RefGvnLiteMsg
from std_msgs.msg import Header

from ros_gvn_marker_utils import create_start_marker, create_goal_marker
from ros_gvn_marker_utils import create_gvn2d_marker
from ros_gvn_marker_utils import create_gvn_dir_marker
import signal
import sys


class RefGvnWrapper:
    def __init__(self, config_table):
        """ Init RefGvnWrapper class.

            This controller subscribes:
                odom from (localization )
                path from (simplified path from A star 2D planner on inflated occupandcy grid)

            Publish:
                desired robot states (zg)
        """
        self.pre = None
        self.core = None
        self.post = None

        # loading external config parameters
        self._config_dict = config_table

        # -------------------- Constants -------------------------
        self._nPath = 2

        # ------------------- ROS Interfaces  --------------------
        self.marker_pub = rospy.Publisher("~vis_marker_array", MarkerArray, queue_size=1)
        self.gvn_status_pub = rospy.Publisher("/ref_gvn_status", RefGvnLiteMsg, queue_size=1)

        # ------------------- Init Modules  --------------------
        self.init_preprocessor()
        self.init_core()
        self.init_postprocessor()
        self.init_markers()

        # set numpy array console print precision = 2
        np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

        rospy.loginfo("REFERENCE GOVERNOR ROS NODE INIT SUCCESFUL!")

        # ------------------- Cached Property --------------------
        self._last_gvn = None
        self._base_marker = None  # constant created once
        self.start2d = None
        self.goal2d = None
        self.gvn_restart = False
        self.gvn_no_intersection = False
        self.localization_fail = False

    def get_2d_loc(self, xvec):
        """
        Get 2d location from robot states, or governor states
        """
        return xvec[0:self._nPath]

    def init_preprocessor(self):
        """
        Init preprocessor of ref_gvn.
        """
        self.pre = RefGvnPreprocess()

    def init_core(self):
        """
        Init ref_gvn core object. The core class need g0, z0, dist0, goal_loc.
        More details in ref_core class internal init function.
        """
        z0 = self.pre._np_z
        nav_path0 = self.pre._np_path
        goal2d = nav_path0[-1]  # last waypoint as goal_loc2d
        # g0 = z0  # both gvn and rbt are in SE(2)
        self._dt = 1.0 / self._config_dict["ctrl_freq"]

        self.core = RefGvnLiteCore(
            z0=z0,
            nav_path0=nav_path0,
            goal2d=goal2d,
            dt=self._dt
        )

        # check if path dimension is compatible with ref_gvn
        if self._nPath != self.core._nPath:
            raise ValueError("self._nPath = %d != ref_gvn._nPath = %d" % (self._nPath, self.core._nPath))

        # fill cached container
        self.start2d = self.get_2d_loc(z0)
        self.goal2d = goal2d

    def init_postprocessor(self):
        """
        Init preprocessor of ref_gvn.
        """
        self.post = RefGvnPostprocess()

    def init_markers(self):
        """
        Create a bunch of marker for rviz visualization of ref_gvn.
        """
        # ------------ create markers for each object ---------------
        # for start and goal
        self.mk_start = create_start_marker(self.start2d)
        self.mk_goal = create_goal_marker(self.goal2d)

        # for governor and local projected goal location
        gvn2d = self.get_2d_loc(self.core.g)
        self.mk_gvn2d = create_gvn2d_marker(gvn2d)

        # arrows gvn2d --> lpg2d
        self.mk_gvn_dir = create_gvn_dir_marker(self.get_2d_loc(self.pre._np_z), gvn2d)

        # assemble markers in makerArray
        self.markers = [self.mk_start, self.mk_goal]
        # self.markers += [self.mk_gvn2d, self.mk_lpg2d, self.mk_ls, self.mk_lf, self.mk_gvn_dir]
        self.markers += [self.mk_gvn2d, self.mk_gvn_dir]

        # ------------ publish marker array---------------
        self.marker_pub.publish(self.markers)

    def show_debug_info(self, yaw_idx=2):
        """
        Display loop debug info.
        """

        g = self.core.g
        s = self.core.progress_rate

        rospy.loginfo_throttle(0.5, "progress rate = %.2f " % s)
        rospy.loginfo_throttle(0.5, " g = [%.2f, %.2f, %.2f (deg)]" % (g[0], g[1], np.rad2deg(g[yaw_idx])))

    def update_moving_markers(self):
        """
        Update non static markers.
        http://wiki.ros.org/rviz/DisplayTypes/Marker#Sphere_.28SPHERE.3D2.29

        Notes: Becareful on scale definition for different type of markers.
        """

        rbt_pose2d = self.get_2d_loc(self.pre._np_z)
        gvn2d = self.get_2d_loc(self.core.g)

        # update mk_gvn2d
        self.mk_gvn2d.pose.position.x = gvn2d[0]
        self.mk_gvn2d.pose.position.y = gvn2d[1]

        # update mk_gvn_arrow
        self.mk_gvn_dir.points[0] = Point(rbt_pose2d[0], rbt_pose2d[1], 0.0)
        self.mk_gvn_dir.points[1] = Point(gvn2d[0], gvn2d[1], 0.0)

        # ---------------- re-publish marker array ----------------
        self.marker_pub.publish(self.markers)

    def publish_gvn_state(self):
        """
        Publish governor states.
        """
        # Init Msg Header
        msg = RefGvnLiteMsg()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()

        if self.pre.localization_fail:
            self.localization_fail = True
        else:
            self.localization_fail = False

        # Robot Pose2D
        msg.rbt_pose2D = Pose2D(x=self.pre._np_z[0], y=self.pre._np_z[1], theta=self.pre._np_z[2])

        # Governor Pose2D
        msg.gvn_pose2D = Pose2D(x=self.core.g[0], y=self.core.g[1], theta=self.core.g[2])

        # Governor Status
        msg.gvn_status = self.core.gvn_status

        # Publish Msg
        self.gvn_status_pub.publish(msg)

    def gvn_restart_check(self):
        """
        In main loop, check if we need to restart governor.
        """
        if not self.pre.gvn_restart_received:
            return
        rospy.logwarn_throttle(1.0, "[ref_gvn_node] >>>>>>>> restart received <<<<<<<<")
        goal_updated = False
        # make sure new path is received after gvn restart requested
        ros_time_gap = self.pre.path_received_time - self.pre.gvn_restart_received_time
        # wait approximately 0.5 seconds
        if self.core.gvn_status >= self.core.IDLE:
            if ros_time_gap.secs > 1:
                goal_updated = True
                if np.size(self.pre._np_path) != 0:
                    self.goal2d = self.pre._np_path[-1]
                    self.mk_goal = create_goal_marker(self.goal2d)
                    self.core._goal2d = self.goal2d
                else:
                    self.mk_goal = create_goal_marker(self.pre._np_z)
                    self.core._goal2d = self.pre._np_z

                self.markers[1] = self.mk_goal
                # if task finished (100), or gvn is IDLE (1) you can approve ref gvn restart request
                if self.core.gvn_status == self.core.GOAL_REACHED or self.core.gvn_status == self.core.IDLE:
                    self.gvn_restart = True
                    # this request is completed
                    self.pre.gvn_restart_received = False
                    rospy.logwarn("[ref_gvn_node] >>>>>>>> restart approved <<<<<<<<")

                # when gvn is running at NORMAL (50), only update goal, deny restart request
                if goal_updated and self.core.gvn_status == self.core.NORMAL:
                    self.pre.gvn_restart_received = False
                    rospy.logwarn("[ref_gvn_node] >>>>>>>> restart denied <<<<<<<<")
        else:
            self.pre.gvn_restart_received = False
            rospy.logwarn("[ref_gvn_node] >>>>>>>> restart denied <<<<<<<<")

    def update(self):
        """
        Update reference as follows:
            1. collect latest data from preprocessor (callback automatically)
            2. execuate update loop using core
                    update(self, dIcO_sq, z, r, debug=True):
            3. sending new desired robot states to downstream
        """

        z = self.pre._np_z

        # extract path from pre-precessor
        r = self.pre._np_path

        # Check if Ref_Gvn Restart Needed
        self.gvn_restart_check()

        # if localization fail, then restart immediately
        if self.localization_fail:
            rospy.logerr_throttle(1.0, "de-localization detected, please restart experiment mannually")

        # gvn restart program
        if self.gvn_restart:
            self.core = None
            self._last_gvn = None
            self.start2d = None
            self.goal2d = None
            self.init_core()
            self.init_markers()
            self.gvn_restart = False
            rospy.logwarn_throttle(0.5, "[ref_gvn_node] ------------ RESTARTED ------------")
        else:
            gvn_state_msg = "[%.2f, %.2f %.2f(deg)]" % (self.core.g[0], self.core.g[1], np.rad2deg(self.core.g[2]))

            rospy.logwarn_throttle(0.5, "[ref_gvn_node] ------------  RUNNING  ------------")
            rospy.loginfo_throttle(0.5, "gvn_status = %d, progress_rate = %.3f" % (self.core.gvn_status, self.core.progress_rate))
            rospy.loginfo_throttle(0.5, "gvn_state = %s" % gvn_state_msg)# rospy.loginfo_throttle(0.5, "upstream")
            rospy.loginfo_throttle(0.5, "z = [%.2f, %.2f, %.2f (deg)]" % (z[0], z[1], np.rad2deg(z[2])))

            if self.core.gvn_status > self.core.IDLE:
                if not self.core.gvn_goal_reached_flag:
                    self.core.update(path=r, z=z, path_update=self.pre.path_updated, debug=False)
                    # show ref_gvn debug info
                    self.show_debug_info()

                    # next governor on unknown, use previous one
                    if not self.pre.gvn_located_on_unknown_flag:
                        self._last_gvn = self.core.g
                    else:
                        self.core.g = self._last_gvn

                    self.post.send_cmd(gvn=self.core.g)
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | ref_gvn_core] ------    NORMAL    ------")
                else:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | ref_gvn_core] ------ GOAL REACHED ------")
                    self.post.send_cmd(gvn=self.core.g)
                    self.core.update(path=r, z=z, path_update=self.pre.path_updated, debug=False)
            else:
                self.post.send_cmd(gvn=z)
                self.core.update(path=r, z=z, path_update=self.pre.path_updated, debug=False)
                # show ref_gvn debug info
                self.show_debug_info()

                # check governor status
                if self.core.gvn_status == self.core.IDLE:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | ref_gvn_core] ------     IDLE     ------")
                else:
                    rospy.logwarn_throttle(0.5, "[ref_gvn_node | ref_gvn_core] ------     ERROR     ------")

        # all update done, reset path updated flag in pre_process
        self.pre.path_updated = False

        # publish governor state
        self.publish_gvn_state()

        # update moving markers in rviz
        self.update_moving_markers()


if __name__ == '__main__':

    config_dict = {'ctrl_freq': 50.0}

    try:
        rospy.init_node('ref_gvn')
        rospy.loginfo("Node Reference Governor Started!\n")

        # set node frequency by reading parameter server
        config_dict['ctrl_freq'] = rospy.get_param("~ctrl_freq", 50.0)
        # gvn control gain
        config_dict['kg'] = rospy.get_param("~kg", 1.0)

        ref_gvn_ros = RefGvnWrapper(config_table=config_dict)
        rate = rospy.Rate(config_dict['ctrl_freq'])

        # signal.signal(signal.SIGINT, ref_gvn_ros.pre.save_planned_path_to_csv)

        # rate = rospy.Rate(ref_gvn_ros.core.rate)
        while not rospy.is_shutdown():
            ref_gvn_ros.update()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
