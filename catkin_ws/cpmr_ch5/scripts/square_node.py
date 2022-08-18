#!/usr/bin/env python3
import rospy
import roslib
import math

from geometry_msgs.msg import Twist

class Square:
  def __init__(self, topic='cmd_vel'):
    self._pub = rospy.Publisher(topic, Twist, queue_size=1, latch=True)

  def do_square(self, rot_vel=0.8, trans_vel=0.2, side_len=2):
    self.cleanup()
    for j in range(4):
      rospy.loginfo(f"Moving forward {side_len} in {side_len/trans_vel}s")
      twist = Twist()
      twist.linear.x = trans_vel
      self._pub.publish(twist)
      rospy.sleep(side_len/trans_vel)
      twist = Twist()
      twist.angular.z = rot_vel
      self._pub.publish(twist)
      rospy.loginfo(f"Rotating 90 degrees in {(3.1415/2)/rot_vel}s")
      rospy.sleep((3.1415/2)/rot_vel)
    self.cleanup()

  def cleanup(self):
    twist = Twist()
    self._pub.publish(twist)

if __name__=="__main__":
  rospy.init_node('square')
  square = Square()
  try:
    square.do_square()
  except rospy.ROSInterruptException:
    pass
  square.cleanup()


