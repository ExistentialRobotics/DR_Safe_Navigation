#!/usr/bin/env python
""" Safe path-following controller based on reference governor

Interfaces:
    Input:
      nonlinear robot states (z), SE(2)
      strictly feasible path (r), 2d
      dist from governor to observed obstalce space (d_Q(g,O))
    Output:
      governor state (g) SE(2)

      # for visualization purpose
        local projected goal (lpg2d), 2d
        local safe zone (LS) ball/ellipse characteristics

"""

from __future__ import print_function

import numpy as np
import numpy.linalg as npla
import rospy
from sensor_msgs.msg import Joy


def compute_segment_lengths(path):
    """
    Obtain length of an piece wise linear path
    """
    lengths = []
    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        lengths.append(np.linalg.norm(p2 - p1))
    return lengths


def get_current_segment(progress_rate, path):
    relative_s = progress_rate * sum(compute_segment_lengths(path=path))
    accumulated_length = 0
    for i, length in enumerate(compute_segment_lengths(path=path)):
        if accumulated_length + length >= relative_s:
            return i, (relative_s - accumulated_length) / length
        accumulated_length += length


def compute_gamma_s(segment_rate, segment_index, path):
    p_start = path[segment_index]
    p_end = path[segment_index + 1]
    return p_start + segment_rate * (p_end - p_start)


class FindIntrError(Exception):
    """ User Defined Exceptions for FindIntr Module.
    """

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "FindIntr exception: {0}".format(self.msg)
        else:
            return "FindIntr exception"


def ball_and_path(x, r, path):
    """
    Find the furthest intersection between a ball and a path in 2D/3D. (d=2/3)
    Inputs:
        x: coordinates of the ball center (d, )
        r: radius of the ball
        path: a series of waypoints (num_pts, d)
    Output
        status: running of this algorithm
                -2: failed
                -1: path reduced to point
                0: succeeded

        x_star  : furthest intersection (d, )
        B_idx: the furthest index of path xg lies in
                Example: B_idx = 6
                 (0)               (5)                (6)
                 START---->....---> A-----x_star-----> B----> .... ---> END

    Ref: personal notes
    """
    # default return
    status, x_star, B_idx = -2, [], []

    plan_fail = False
    if len(path) < 2:
        path = np.concatenate((path, path), axis=0)
        # raise FindIntrError("path len is too short, >=2 waypoints is required!")

    # Test if planning failed
    if np.size(path) == 4:
        identical_criteria = path[0, :] == path[1, :]
        if np.all(identical_criteria):
            plan_fail = True

    # loop backward
    if plan_fail:
        B_idx = -1
        x_star = path[1, :]
        status = -1
    else:
        for path_idx in range(1, len(path)):
            B = path[path_idx]
            A = path[path_idx - 1]
            B_idx = path_idx
            dAB = npla.norm(A - B)  # segment length ptA from ptB
            u = x - A
            v = B - A
            w1 = np.inner(u, v) / dAB ** 2
            w1hat = max(min(w1, 1), 0)  # normalized to [0,1]
            dist2segAB = npla.norm(u - w1hat * v)

            if dist2segAB > r:
                if status == 0:
                    B_idx = path_idx - 1
                break
            else:
                # distance from x to line AB
                dist2lineAB = npla.norm(u - w1 * v)
                # print('DEBUG dist to line AB is %.2f' %(dist2lineAB))
                w2 = np.sqrt(r ** 2 - dist2lineAB ** 2) / dAB  # ratio of |v|
                w = w1 + w2
                w = max(min(w, 1), 0)
                x_star = (1 - w) * A + w * B
                status = 0

    return status, x_star, B_idx


class RefGvnError(Exception):
    """User Defined Exceptions for Reference Governor class."""

    def __init__(self, *args):
        if args:
            self.msg = args[0]
        else:
            self.msg = ""

    def __str__(self):
        if self.msg:
            return "RefGvn exception: %s" % self.msg
        else:
            return "RefGvn exception"


def wrap_angle_pmp(angle_vec):
    """
    npla.normalize angle in radian to [-pi, pi)
    angle_vec: angle despcription in radian
    """
    angle_vec = (angle_vec + np.pi) % (2 * np.pi) - np.pi
    return angle_vec


class RefGvnLiteCore:
    """
    Reference Governor Core Module. Responsible for:
        1) local projected goal (lpg) update
        2) governor state update (g)
    Update governor sates.
        Normal
        Halt
        Goal reached
        ...
    """

    # Running status table, higher number better status
    IDLE = 0
    NORMAL = 50
    GOAL_REACHED = 100
    INIT_FAILED = -10

    def __init__(
            self,
            z0,  # init robot state, in SE(2), represented in (3,)
            nav_path0,  # init nav path in 2d, represented in (num waypoints, 2)
            goal2d,  # goal 2d location
            dt=0.02,  # time step
            gvn2goal_th=0.01,  # governor to goal positional threshold in meter
            nPath=2  # governor to goal positional threshold in meter
    ):
        """
        Reference governor object init. Load parameters.
        """

        # system information init ----------------------------------------------
        # system states
        self.z = z0
        self.progress_rate = 0.00
        self.g = z0
        self.g2G = 0.00
        self._nPath = nPath  # dimensional of path, num of positional states in z
        self._goal2d = goal2d

        # static parameters
        self._dt = dt
        self._gvn2G_th = gvn2goal_th

        # cached large containers ----------------------------------------------
        self.path_raw = nav_path0
        self.length_ref = 8

        # local flag -----------------------------------------------------------
        self.gvn_goal_reached_flag = False
        self.plan_fail = True
        self.human_override = False

        # perform init checklist- ----------------------------------------------
        # Init status to normal for a full check
        self.gvn_status = RefGvnLiteCore.NORMAL
        # Perform status check
        self.ref_gvn_core_status_check()

        # strict safety metric at init
        if self.gvn_status >= RefGvnLiteCore.IDLE:
            rospy.logwarn("[RefGvnSE2Core] ref_gvn init safely")
        else:
            err_msg = "Init robot state z0 = %s is unsafe" % z0
            self.gvn_status = RefGvnLiteCore.INIT_FAILED
            rospy.logerr_throttle(0.5, err_msg)

        # ------------------- ROS Subscribers  --------------------
        self.human_sub = rospy.Subscriber("/bluetooth_teleop/joy", Joy, self.human_override_callback, queue_size=1)

    def human_override_callback(self, msg):
        """
        Human override status set function, when the L1 button on the joystick is pressed, it will set
        the human_override flag to true.
        If the human_override flag is true, ref_gvn_status will be set to IDLE, but it will not send an
        emergency stop signal.
        """
        if msg.buttons[4] == 1:
            self.human_override = True
        else:
            self.human_override = False

    def update_gvn_state(self):
        """
        Governor update
        """
        # Compute the current segment and relative s
        segment_index, relative_s = get_current_segment(progress_rate=self.progress_rate, path=self.path_raw)

        # Compute gamma_s
        gamma_s = compute_gamma_s(segment_rate=relative_s, segment_index=segment_index, path=self.path_raw)

        # Compute distance to reference
        
        '''
        if the car got stuck at the corners (possibly becasue the reference point is on the other side of the wall), you could add a constant (0.1 ~ 0.4) to shorten the distance between the robot and the reference to help, 
        However, this may degrade the tracking perforamce, and requires tuning the control parameters to avoid chattering behaviours around the reference path. 
        '''
        distance_to_ref = np.linalg.norm(self.z[0:2] - gamma_s) 

        # Path length scaling
        p_scal = self.length_ref / sum(compute_segment_lengths(path=self.path_raw))
        # p_scal = 1

        # Adaptive reference governor dynamics
        k = 0.25 * p_scal / (1 + distance_to_ref ** 10)

        gvn_projection_range_limit = 2

        if npla.norm(self.z[0:2] - self.g[0:2]) <= gvn_projection_range_limit:
            self.progress_rate += self._dt * k * (1 - self.progress_rate ** 15)
            segment_index, relative_s = get_current_segment(progress_rate=self.progress_rate, path=self.path_raw)
            g_new_xy = compute_gamma_s(segment_rate=relative_s, segment_index=segment_index, path=self.path_raw)
            g_new = np.append(g_new_xy, 0)
        else:
            g_new = self.g
            rospy.logwarn_throttle(0.5, "[ref_gvn_core | gvn too far > %.2f]" % gvn_projection_range_limit)

        self.g = g_new

    def check_gvn_goal_reached(self):
        """
        Check whether goal is reached for governor
        Output/Update:
            Internal state gvn_status
            g2G, gvn to goal in 2 norm
        """
        gvn2d = self.g[0:self._nPath]
        # eculidean distance from governor position to goal position
        self.g2G = npla.norm(gvn2d - self._goal2d)
        if self.g2G < self._gvn2G_th and not self.gvn_goal_reached_flag:
            print("\n------------ Governor Reached Goal Region! -----------\n")
            self.gvn_goal_reached_flag = True
            self.gvn_status = RefGvnLiteCore.GOAL_REACHED

    def path_validation(self):
        """
        Check if path is valid
        """
        self.plan_fail = True
        # Test if path dimension valid
        if len(self.path_raw) < 2:
            print(self.path_raw)
            rospy.logerr_throttle(1.0, "path len is too short, >=2 waypoints is required!")
        # Test if planning failed
        elif np.size(self.path_raw) == 4:
            # if start loc equals to goal loc (hacked in A* planning for fail case. But this is not correct.)
            # sometimes A* success, with same start and goal. But has waypoint in path.
            if np.all(self.path_raw[0, :] == self.path_raw[1, :]):
                self.plan_fail = True
            else:
                self.plan_fail = False
        else:
            self.plan_fail = False

    def ref_gvn_core_status_check(self):
        """
        Run status change for ref_gvn_core, no status change outside this function.
        """

        # GOAL_REACHED Case, no action take
        if self.human_override:
            self.gvn_status = RefGvnLiteCore.IDLE
            rospy.logwarn_throttle(0.5, "[ref_gvn_core] ---- HUMAN OVERRIDE | DRIVE SAFE ----")
            return

        # GOAL_REACHED Case, no action take
        if self.gvn_status == RefGvnLiteCore.GOAL_REACHED:
            self.gvn_status = RefGvnLiteCore.GOAL_REACHED
            rospy.logwarn_throttle(0.5, "[ref_gvn_core] ------------ GOAL REACHED ------------")

        # NORMAL Case, run status table
        elif self.gvn_status == RefGvnLiteCore.NORMAL:
            # Check path to set flag: self.plan_fail
            self.path_validation()

            # IDLE Case
            if self.plan_fail:
                self.gvn_status = RefGvnLiteCore.IDLE
                rospy.loginfo("Warning Planning Failed, set gvn_status = IDLE")
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] NORMAL ==========> IDLE")
                return

            # If planning not fail, check if reach goal to set flag: self.gvn_goal_reached_flag
            self.check_gvn_goal_reached()

            # GOAL REACHED Case
            if self.gvn_goal_reached_flag:
                # Regular Case -> Gvn Goal Reached
                self.gvn_status = RefGvnLiteCore.GOAL_REACHED
                rospy.loginfo("Governor Reach Goal")
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] NORMAL ==========> GOAL_REACHED")
            # NORMAL Case
            else:
                # Regular Case && Gvn Goal not Reached
                self.gvn_status = RefGvnLiteCore.NORMAL
                rospy.logwarn_throttle(0.5, "[ref_gvn_core] ------------ NORMAL ------------")

        # IDLE Case, no action take, keep staying at IDLE
        elif self.gvn_status == RefGvnLiteCore.IDLE:
            self.gvn_status = RefGvnLiteCore.IDLE
            rospy.logwarn_throttle(0.5, "[ref_gvn_core] ------------ IDLE ------------")

    def update_progress_rate(self, rbt_pose, path):
        """
        This function updates progress rate s to reflect changes in planning path
        """
        # Obtain current governor to robot distance
        governor_to_robot_dist = npla.norm(rbt_pose[0:2] - self.g[0:2])

        # obtain new governor position on path
        status, x_star, end_point_index = ball_and_path(x=self.z[0:2], r=governor_to_robot_dist, path=path)

        # obtain segment lengths
        lengths = compute_segment_lengths(path=path)

        # update progress rate
        if status == 0:
            self.progress_rate = (npla.norm(x_star - path[end_point_index-1]) + sum(lengths[0:end_point_index-1])) / sum(lengths)
        else:
            self.progress_rate = 0

    def update(self, z, path, path_update=False, debug=True):
        """
        Update reference signal gvec according to dynamic safety margin
        deltaE feedback and governor running status.
        Input:
            @dist: distance metric for internal safety metric computation
            @z: robot states
            @r: reference path
        Output/Update:
            @gvn_status: reference governor running status
            @self.gbar: local projected goal
        """
        # load parameters from input args
        self.z = z

        # perform status check
        self.ref_gvn_core_status_check()

        # operations based on gvn_status, rough sort ---------------------------
        # status better than IDLE
        if self.gvn_status >= RefGvnLiteCore.IDLE:
            # IDLE Case, set gbar to current g
            if self.gvn_status == RefGvnLiteCore.IDLE:
                self.g = z
            # NORMAL Case, update governor state
            elif self.gvn_status == RefGvnLiteCore.NORMAL:
                self.update_gvn_state()
            # GOAL_REACH Case, set gbar to final goal
            else:
                self.g = np.append(self._goal2d, 0)

        # check if new path available
        if path_update:
            self.update_progress_rate(rbt_pose=self.z, path=path)
            self.path_raw = path

        if debug and self.gvn_status >= RefGvnLiteCore.IDLE:
            print("[Progress %.2f] g2G = %.2f" % (self.progress_rate, self.g2G), end="")
            print(" g =    [%.2f, %.2f, %.2f (deg)]" % (self.g[0], self.g[1], np.rad2deg(self.g[2])), end="")
