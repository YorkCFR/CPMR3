#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class HarrisCorners:
  def __init__(self, image_topic):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)

  def _image_callback(self, msg):
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8").copy()
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners = cv2.cornerHarris(grey, 2, 3, 0.04)
    corners = cv2.dilate(corners, None)
    image[corners > 0.01 * corners.max()] = [255, 0, 0]
    cv2.imshow('corners', image)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('harris_corners')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")

  x = HarrisCorners(image_topic)
  rospy.spin()
