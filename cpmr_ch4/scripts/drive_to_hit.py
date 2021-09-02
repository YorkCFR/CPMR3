#! /usr/bin/env python3
# 
# Drive a robot equipped with a bumper along x until it hits something
# then backup at the same velocity for 5 seconds
#
# Copryight (c) Michael Jenkin, 2021
#
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ContactsState

contact = None
odom = None

def odometry_callback(msg):
  global odom
  odom = msg

def contact_callback(msg):
  global contact
  contact = msg

if __name__ == '__main__':
  rospy.init_node('drive_to_hit')
  robot = rospy.get_param("~robot", "block_robot")
  cmdvel = rospy.get_param("~cmdvel", "cmd_vel")
  backup_time = float(rospy.get_param("~backupt", "5.0"))
  velocity = float(rospy.get_param("~velocity", "0.5"))
  odomMsg = rospy.get_param("~odom", "odom")
  contactMsg = rospy.get_param("~contact", "robot_bumper_contact_state")
  rate = int(rospy.get_param("~rate", "30"))


  r =rospy.Rate(rate) # in hz
  rospy.Subscriber(odomMsg, Odometry, odometry_callback)
  rospy.Subscriber(contactMsg, ContactsState, contact_callback)

  rospy.loginfo(f"Starting to publish on {cmdvel}")
  pub = rospy.Publisher(cmdvel, Twist, queue_size=1)

  twist = Twist()
  twist.linear.x = velocity
  pub.publish(twist)

  try:
    while not rospy.is_shutdown():
      pub.publish(twist)
      r.sleep()
      posMsg = ""
      if odom != None:
        posMsg = f"Robot at ({odom.pose.pose.position.x},{odom.pose.pose.position.y})"
      contactMsg = ""
      if contact != None:
        if len(contact.states) == 0:
          contactMsg = "No contact"
        else:
          contactMsg = f"Contact positions {contact.states[0].contact_positions}"
          twist = Twist()
          twist.linear.x = -velocity
          rospy.loginfo(f"Backing up")
      rospy.loginfo(posMsg + " " + contactMsg)
        
  except rospy.ROSInterruptException:
    pass
  twist = Twist()
  pub.publish(twist)
  rospy.loginfo(f"Shutting down")
