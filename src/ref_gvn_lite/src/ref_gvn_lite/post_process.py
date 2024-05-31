#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose2D


class RefGvnPostprocess:
    """ 
    Reference Governor Postprocess Module. Responsible for:
        1) optional state transformation, e.g., linear->nonlinear, polar->cartesian
        2) send out desired waypoint for robot z* = z_g = g
        3) visualization markers (g, lpg2d, LS, etc.)

    """

    # Running status table, higher number better status

    def __init__(self):

        # publishers
        # publish immediate goal (zg) for robot control
        self._zg_pub = rospy.Publisher('~local_goal', Pose2D, queue_size=1)
        # message for rbt_state_dsr_pub
        self._zg = Pose2D()
        rospy.loginfo("Node [ref_gvn_postprocess] ready \n")

    def send_cmd(self, gvn):
        """
        Publish robot command to low level controller. 
        For SE(2) robot z* = z_g = gbar 
        """
        self._zg.x = gvn[0]
        self._zg.y = gvn[1]
        self._zg.theta = gvn[2]
        self._zg_pub.publish(self._zg)
        rospy.logdebug("[ref_gvn_post] Publish local goal Pose2D [%.2f, %.2f, %.2f (deg)]" % (self._zg.x, self._zg.y, np.rad2deg(self._zg.theta)))
        return
