#!/usr/bin/env python3

"""
Put ROS message utility functions. 
"""

import numpy as np
from matplotlib import colors as mcolors

import rospy
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


def get_base_marker(loc, rpy_angles=None, frame_id="map"):
    """
    Get a base marker object for ROS rviz display.
    """

    base_marker = Marker()
    base_marker.header.frame_id = frame_id
    # Setting stamp to time(0) shows the marker as it is published, without matching the time
    base_marker.header.stamp = rospy.Time(0)

    # Set the scale of the ref_marker
    base_marker.scale.x = 1.0
    base_marker.scale.y = 1.0
    base_marker.scale.z = 1.0

    # set marker position
    base_marker.pose.position.x = loc[0]
    base_marker.pose.position.y = loc[1]
    if len(loc) == 3:
        base_marker.pose.position.z = loc[2]

    # set marker pose
    if rpy_angles is None:
        base_marker.pose.orientation.x = 0.0
        base_marker.pose.orientation.y = 0.0
        base_marker.pose.orientation.z = 0.0
        base_marker.pose.orientation.w = 1.0
    else:
        # !!! use this function carefully angles are in rad, sxyz axis
        q = quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])
        base_marker.pose.orientation.x = q[0]
        base_marker.pose.orientation.y = q[1]
        base_marker.pose.orientation.z = q[2]
        base_marker.pose.orientation.w = q[3]

    return base_marker


def create_actor2d(id, type, loc2d, disp_h, color_rgba, frame_id='map', yaw=None, scale_xyz=[1.0, 1.0, 1.0]):
    """
    Create a 2d actor from base_marker (rviz marker)
    Input:
      @loc2d: center of object
      @disp_h: rendering height for better visualization
      @rpy_angles: object orientation
      @id: marker id, unique int
    """
    loc = np.hstack((loc2d, disp_h))
    # rpy_angles = np.array([0.0, 0.0, yaw])

    marker = Marker()
    marker.id = id
    marker.type = type
    marker.header.frame_id = frame_id
    # Setting stamp to time(0) shows the marker as it is published, without matching the time
    marker.header.stamp = rospy.Time(0)

    # set marker pose
    # set position
    marker.pose.position.x = loc[0]
    marker.pose.position.y = loc[1]
    marker.pose.position.z = disp_h

    # set pose
    # !!! use this function carefully angles are in rad, sxyz axis
    if yaw is not None:
        q = quaternion_from_euler(0.0, 0.0, yaw)
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
    else:
        # default pose constructor is q(0, 0, 0, 0) is invalid
        marker.pose.orientation.w = 1.0

        # Color choices: https://www.december.com/html/spec/colorrgbadec.html
    marker.color.r = color_rgba[0]
    marker.color.g = color_rgba[1]
    marker.color.b = color_rgba[2]
    marker.color.a = color_rgba[3]

    # for different object type the meaning of scale is different
    # A scale of [1,1,1] means the object will be 1m X 1m X 1m.
    marker.scale.x = scale_xyz[0]
    marker.scale.y = scale_xyz[1]
    marker.scale.z = scale_xyz[2]

    return marker


def create_point2d_marker(loc2d, id, disp_h=-0.1, color_rgba=[0.0, 1.0, 0.0, 1.0]):
    """
    Create marker for 2d point.
    Input:
      @id: unique marker id
      @disp_h: display height of 2d marker
    """
    point2d_mk = create_actor2d(
        id=id,
        type=1,
        loc2d=loc2d,
        disp_h=disp_h,
        color_rgba=color_rgba,
        scale_xyz=[0.2, 0.2, 0.1],
    )
    return point2d_mk


def create_arrow2d_marker(start2d, goal2d, id, disp_h=0.0):
    """
    Create marker for start2d --> goal2d
    Purple arrow (using arrow, type 0)
    """
    start_point3d = Point(start2d[0], start2d[1], 0.0)
    end_point3d = Point(goal2d[0], goal2d[1], 0.0)

    arrow2d_mk = create_actor2d(
        id=id,
        type=0,
        loc2d=np.zeros(2),  # must be all zeros otherwise arrow will be shifted accordingly
        disp_h=disp_h,
        color_rgba=[0.5, 0.0, 0.5, 1.0],
        scale_xyz=[0.08, 0.2, 0.3],  # shaft diameter, head diameter, head length
    )
    # using points to specify arrow, the pose of marker is not used
    # http://wiki.ros.org/rviz/DisplayTypes/Marker#Sphere_.28SPHERE.3D2.29
    arrow2d_mk.points = [start_point3d, end_point3d]

    return arrow2d_mk


def create_arrow_from_pose(pose2d):
    """
    Create marker for gvn2d --> lpg2d
    Purple arrow (using arrow, type 0)
    """
    # using pose to specify arrow
    # Pivot point is around the tip of its tail. 
    # Identity orientation points it along the +X axis. 
    # scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.

    arrow_actor = create_actor2d(
        id=6,
        type=0,
        loc2d=np.array([pose2d.x, pose2d.y]),
        disp_h=0.0,
        color_rgba=[0.0, 0.0, 1.0, 1.0],
        scale_xyz=[1.2, 0.05, 0.05],
    )

    return arrow_actor


def create_cylinder2d_marker(id, loc2d, disp_h=0.1, c='orange', alpha=0.8, r=0.2, ns='myCylinder'):
    """
    Create marker for 2d circle in rviz. (radius:r)
    Small blue ball (using cylinder type 3)
    """
    cylinder2d_actor = create_actor2d(
        id=id,
        type=3,
        loc2d=loc2d[0:2],
        disp_h=disp_h,
        color_rgba=mcolors.to_rgba(c, alpha),
        scale_xyz=[r * 2.0, r * 2.0, 0.1],  # diameter x, diameter y, height
    )
    cylinder2d_actor.ns = ns
    return cylinder2d_actor


def create_ellipsoid_marker(np_pt, axes_a_heading, axes_a, axes_b, disp_h=0.0, c='orange', ns='myEllipsoid'):
    """
    Create marker for 2d point in rviz.
    Small blue ellipsoid (using cylinder type 3)
    """
    ellipsoid2d_actor = create_actor2d(
        id=0,
        type=3,
        loc2d=np_pt[0:2],
        disp_h=disp_h,
        color_rgba=mcolors.to_rgba(c, 0.8),
        scale_xyz=[axes_a, axes_b, 0.1],  # diameter x, diameter y, height
        yaw=axes_a_heading,
    )
    ellipsoid2d_actor.ns = ns
    return ellipsoid2d_actor


def create_pose_arrow_marker(id, start2d, goal2d, disp_h=0.1, c='grey', alpha=0.8):
    """
    Create marker for start2d --> goal2d
    Purple arrow (using arrow, type 0)
    """
    start_point3d = Point(start2d[0], start2d[1], 0.0)
    end_point3d = Point(goal2d[0], goal2d[1], 0.0)

    arrow2d_mk = create_actor2d(
        id=id,
        type=0,
        loc2d=np.zeros(2),  # must be all zeros otherwise arrow will be shifted accordingly
        disp_h=disp_h,
        color_rgba=mcolors.to_rgba(c, alpha),
        scale_xyz=[0.08, 0.2, 0.3],  # shaft diameter, head diameter, head length
    )
    # using points to specify arrow, the pose of marker is not used
    # http://wiki.ros.org/rviz/DisplayTypes/Marker#Sphere_.28SPHERE.3D2.29
    arrow2d_mk.points = [start_point3d, end_point3d]
    return arrow2d_mk
