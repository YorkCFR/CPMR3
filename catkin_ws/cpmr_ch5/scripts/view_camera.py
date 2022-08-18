#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

class ViewCamera:
  def __init__(self, image_topic):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)

  def _image_callback(self, msg):
    image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
    print(image.shape)
    cv2.imshow('window', image)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('view_camera')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")
  print(image_topic)

  x = ViewCamera(image_topic)
  rospy.spin()
