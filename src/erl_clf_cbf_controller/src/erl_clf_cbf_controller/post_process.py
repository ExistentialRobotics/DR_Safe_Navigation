#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, Header
from tf.transformations import quaternion_from_euler


def clip(x, x_min, x_max):
    """
    clip x in [x_min, xmax]
    """
    if x < x_min:
        x = x_min
    if x > x_max:
        x = x_max
    return x


class ClfCbfPostprocess:
    """
    Create downstream publisher interface
    Optional state transformation, e.g., linear->nonlinear, polar->cartesian
    Clip control with repsect to hardware limits
    """

    def __init__(self, ctrl_limits=None):

        # publishers
        # publish velocity command (body twist) for mobile platform hardware / simulated dynamics
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sdf_val_pub = rospy.Publisher('/sdf_val', Float64, queue_size=1)
        self.debug_msg_pub = rospy.Publisher('/clf_cbf_debug', PoseStamped, queue_size=1)
        self._body_twist = Twist()
        self._sdf_val = Float64()
        self._sdf_grad = PoseStamped()
        self._sdf_grad.header = Header()
        self._sdf_grad.header.frame_id = 'map'
        rospy.loginfo("[unicycle controller post-processor initialized!]")

        if ctrl_limits is not None:
            self.ctrl_limits = ctrl_limits

    def send_cmd(self, v_dsr, w_dsr, clip_ctrl=False, debug=False):
        """
        Publish low level command velocity to hardware or simulated dynamics
        For unicycle-like robot, desired linear and angular velocity
        """
        if debug:
            rospy.logwarn_throttle(0.5, "Input body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr))

        if clip_ctrl and self.ctrl_limits is not None:
            v_dsr = clip(v_dsr, self.ctrl_limits['v_min'], self.ctrl_limits['v_max'])
            w_dsr = clip(w_dsr, self.ctrl_limits['w_min'], self.ctrl_limits['w_max'])

        if debug:
            rospy.logwarn_throttle(0.5, "Output body twist (v_dsr, omega_dsr) [%.2f, %.2f]" % (v_dsr, w_dsr))

        self._body_twist.linear.x = v_dsr
        self._body_twist.angular.z = w_dsr
        self.cmd_vel_pub.publish(self._body_twist)

    def send_sdf_val(self, sdf_server_result):

        self.sdf_val_pub.publish(sdf_server_result)

    def send_debug(self, rbt_pose, gradient):
        # Header time
        self._sdf_grad.header.stamp = rospy.Time.now()

        # Position
        self._sdf_grad.pose.position.x = rbt_pose[0]
        self._sdf_grad.pose.position.y = rbt_pose[1]
        self._sdf_grad.pose.position.z = 0

        # Quaternion
        quaternion = quaternion_from_euler(0.0, 0.0, np.arctan2(gradient[1], gradient[0]))
        self._sdf_grad.pose.orientation.x = quaternion[0]
        self._sdf_grad.pose.orientation.y = quaternion[1]
        self._sdf_grad.pose.orientation.z = quaternion[2]
        self._sdf_grad.pose.orientation.w = quaternion[3]

        self.debug_msg_pub.publish(self._sdf_grad)
