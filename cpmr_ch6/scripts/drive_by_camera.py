#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class DriveByCamera:
  _FORWARD = 1
  _TURNING_LEFT = 2
  _TURNING_RIGHT = 3
  
  def __init__(self, image_topic, cmd_topic, x_vel, theta_vel, output, image_size):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)
    self._pub = rospy.Publisher(cmd_topic, Twist, queue_size = 1)
    self._x_vel = x_vel
    self._theta_vel = theta_vel
    self._output = output
    self._image_size = image_size
    self._recording = False
    self._curdir = None
    self._left_id = 0
    self._right_id = 0
    self._forward_id = 0

  def _image_callback(self, msg):
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
    cv2.imshow('window', image)
    key = cv2.waitKey(3)

    if key == 106:
      self.turn_left()
      self._curdir = DriveByCamera._TURNING_LEFT
    elif key == 107:
      self.go_straight()
      self._curdir = DriveByCamera._FORWARD
    elif key == 108:
      self.turn_right()
      self._curdir = DriveByCamera._TURNING_RIGHT
    elif key == 32:
      self.stop()
      self._curdir = None
      print(f"No recording when stopped")
    elif key == 115:
      print(f"Start recording to {self._output}")
      self._recording = True
    elif key == 120:
      print(f"Stop recording left {self._left_id} forward {self._forward_id} right {self._right_id}")
      self._recording = False

    if self._recording and (self._curdir != None):
      dim = min(image.shape[0], image.shape[1])
      oy = int((image.shape[0] - dim) / 2)
      ox = int((image.shape[1] - dim) / 2)
      roi = image[oy:oy+dim, ox:ox+dim, 0:3]
      out = cv2.resize(image, (self._image_size, self._image_size))
      cv2.imshow("Resized", out)
      if self._curdir == DriveByCamera._FORWARD:
        cv2.imwrite(f"{self._output}/forward/image.{self._forward_id:08d}.jpg", out)
        self._forward_id = self._forward_id + 1
      elif self._curdir == DriveByCamera._TURNING_LEFT:
        cv2.imwrite(f"{self._output}/left/image.{self._left_id:08d}.jpg", out)
        self._left_id = self._left_id + 1
      elif self._curdir == DriveByCamera._TURNING_RIGHT:
        cv2.imwrite(f"{self._output}/right/image.{self._right_id:08d}.jpg", out)
        self._right_id = self._right_id + 1
      
      

  def _command(self, x_vel, theta_vel):
    twist = Twist()
    twist.linear.x = x_vel
    twist.angular.z = theta_vel
    self._pub.publish(twist)

  def go_straight(self):
    self._command(self._x_vel, 0)

  def turn_left(self):
    self._command(self._x_vel, self._theta_vel)

  def turn_right(self):
    self._command(self._x_vel, -self._theta_vel)

  def stop(self):
    self._command(0, 0)

if __name__ == '__main__':
  rospy.init_node('drive_by_camera')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")
  cmd_topic = rospy.get_param("~cmd", "cmd_vel")
  hz = int(rospy.get_param("~rate", 10))
  output = rospy.get_param("~output", "database")
  x_vel = float(rospy.get_param("~x_vel", 0.2))
  theta_vel = float(rospy.get_param("~theta_vel", 0.2))
  image_size = int(rospy.get_param("~size", 28))

  camera = DriveByCamera(image_topic, cmd_topic, x_vel, theta_vel, output, image_size)
  rospy.spin()
