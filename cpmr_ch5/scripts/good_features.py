#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image
import numpy as np

class GoodFeatures:
  def __init__(self, image_topic):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)

  def _image_callback(self, msg):
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8").copy()
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(grey, 25, 0.01, 10)
    print(corners)
    if not (corners is None):
      corners = np.int0(corners)
      for i in corners:
        x, y = i.ravel()
        cv2.circle(image, (x, y), 3, 255, -1)
    cv2.imshow('corners', image)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('good_features')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")

  x = GoodFeatures(image_topic)
  rospy.spin()
