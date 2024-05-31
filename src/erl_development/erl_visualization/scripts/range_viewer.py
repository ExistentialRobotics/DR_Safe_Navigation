#!/usr/bin/env python

''' This script takes an odometry topic as input and outputs a cylindrical marker indicating a projected field
of view on the ground. The cylinder is centered at the odometry point, and is colored according to the given RGB-A
parameters.'''

import rospy

# ROS Message Types
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

robot_state = []  # Assign the robot_state as a global variable.


def robot_odom_cb(robot_odom):
    '''
    Callback for Robot Odometry messages. Updates the global robot state.
    Args:
        robot_odom: The incoming robot odometry message.

    Returns:
        None
    '''
    if len(robot_state) == 0:
        robot_state.append(robot_odom.pose.pose.position.x)
        robot_state.append(robot_odom.pose.pose.position.y)
    else:
        robot_state[0] = robot_odom.pose.pose.position.x
        robot_state[1] = robot_odom.pose.pose.position.y


def gen_cylinder(idx=0, x=5, y=5, frame='map', range=2, height=.05, alpha=1, r=0, g=0, b=1):
    '''
    Generates an RVIZ Marker for a cylinder with the desired parameters and range.
    Args:
        idx:
        x: The x coordinate of the cylinder center .
        y: The ycoordinate of the cylinder center .
        frame:  The frame that the center is expressed in. Default is to map.
        range: The radius of the cylinder.
        height: The height of the cylinder.
        alpha: The opacity (alpha) of the cylinder.
        r: The r value.
        g: The g value.
        b: The b value.

    Returns: An RViz marker that can be published.

    '''
    marker = Marker()
    marker.header.frame_id = frame
    marker.id = idx
    marker.type = Marker.CYLINDER
    # Initialize or Modify the Cylinder
    marker.action = 0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = height / 2
    marker.pose.orientation.w = 1
    # Assign Scale
    marker.scale.x = 2 * range
    marker.scale.y = 2 * range
    marker.scale.z = height
    # Assign Color
    marker.color.a = alpha
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    return marker


if __name__ == "__main__":

    # Init rospy
    rospy.init_node('range_viewer')

    # Read Parameters
    range = rospy.get_param('~range', 2)
    height = rospy.get_param('~height', .05)
    alpha = rospy.get_param('~alpha', 1)
    r = rospy.get_param('~red', 0)
    g = rospy.get_param('~green', 0)
    b = rospy.get_param('~blue', 1)

    # Setup Subscriber to Odom
    robot_sub = rospy.Subscriber('input', Odometry, robot_odom_cb)
    # Setup Range Publisher
    range_pub = rospy.Publisher('output', Marker, queue_size=10)

    rate = rospy.Rate(30)
    while len(robot_state) == 0:  # Sleep Until ODOM is received to ensure things are set up.
        rate.sleep()

    while not rospy.is_shutdown():
        # Get Optional attack state
        attacked = rospy.get_param('attack_state', False)
        if not attacked:
            range_pub.publish(gen_cylinder(0, robot_state[0], robot_state[1], frame='map', range=range,
                                       height=height, alpha=alpha, r=r, g=g, b=b))
        else:
            range_pub.publish(gen_cylinder(0, robot_state[0], robot_state[1], frame='map', range=range,
                                           height=height, alpha=alpha, r=1, g=0, b=0))

        rate.sleep()
