#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class CannyEdges:
  def __init__(self, image_topic):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)

  def _image_callback(self, msg):
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grey, 100, 200)
    cv2.imshow('input', grey)
    cv2.imshow('edges', edges)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('canny_edges')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")

  x = CannyEdges(image_topic)
  rospy.spin()
