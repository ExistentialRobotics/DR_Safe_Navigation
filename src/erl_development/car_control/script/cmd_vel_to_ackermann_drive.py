#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
max_angle = 0.75
max_speed = 2

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  global max_angle, max_speed
  if omega == 0 or v == 0:
    return 0

  radius = v / omega
  angle = math.atan(wheelbase / radius)
  if angle > max_angle:
    angle = max_angle
  if angle < -max_angle:
    angle = -max_angle
  return angle


def cmd_callback(data):
  global wheelbase
  global ackermann_cmd_topic
  global frame_id
  global pub
  global max_angle, max_speed
  
  v = data.linear.x
  if v > max_speed:
    v = max_speed
  if v < -max_speed:
    v = -max_speed
  steering = convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)
  
  msg = AckermannDriveStamped()
  msg.header.stamp = rospy.Time.now()
  msg.header.frame_id = frame_id
  msg.drive.steering_angle = steering
  msg.drive.speed = v
  
  pub.publish(msg)
  




if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive')
        
    twist_cmd_topic = rospy.get_param('~twist_topic', "/cmd_vel")
    ackermann_cmd_topic = rospy.get_param('~ackermann_topic', "/vesc/ackermann_cmd_mux/input/teleop")
    wheelbase = rospy.get_param('~wheelbase', 0.325)
    frame_id = rospy.get_param('~frame_id', 'odom')
    
    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass

