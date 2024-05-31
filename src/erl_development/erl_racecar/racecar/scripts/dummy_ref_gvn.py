#!/usr/bin/env python
"""
Take in path_msg and publish ackermann msg


Zhichao Li @ ERL UCSD
Oct-22 2021


"""
#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped

seq = 0
dummy_ctrl_end_flag = False


def path_callback(path):
    #print("Received path!")
    rospy.logdebug_throttle(
        1, "dummy controller node -----path received----------")
    # rospy.loginfo("dummy controller node -----path received----------")
    dummy_control()


def dummy_control():
    global dummy_ctrl_end_flag
    global ackermann_cmd_topic
    global frame_id
    global ack_pub
    global seq

    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    # Go straight
    if seq < 200 or 350 < seq <= 500:
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.5
    # Turn left
    if 200 <= seq < 250:
        msg.drive.steering_angle = 0.5
        msg.drive.speed = 0.5
    # Turn right
    if 250 <= seq < 350:
        msg.drive.steering_angle = -0.5
        msg.drive.speed = 0.5
    # Stop
    if seq > 500:
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        dummy_ctrl_end_flag = True
        rospy.logwarn_once(
            "dummy controller node -----dummy ctrl end----------")

    ack_pub.publish(msg)
    if not dummy_ctrl_end_flag:
        rospy.logdebug_throttle(
            1, "dummy controller node -----control published----------")
    seq = seq + 1


if __name__ == '__main__':

    ROS_RATE_DUMMY_CTRL = 30

    try:

        rospy.init_node('dummy_controller', log_level=rospy.DEBUG)
        path_topic = '~path'
        ackermann_cmd_topic = '~ackermann_cmd'
        frame_id = rospy.get_param('~frame_id', 'odom')
        rospy.Subscriber(path_topic, Path, path_callback)
        ack_pub = rospy.Publisher(ackermann_cmd_topic,
                                  AckermannDriveStamped,
                                  queue_size=1)
        rospy.loginfo(
            "Node 'dummy_controller' started.\nListening to %s, publishing to %s. Frame id: %s",
            path_topic, ackermann_cmd_topic, frame_id)

        # Publish control signal at a fixed rate
        rate = rospy.Rate(ROS_RATE_DUMMY_CTRL)
        while not rospy.is_shutdown():
            # gen_control()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
