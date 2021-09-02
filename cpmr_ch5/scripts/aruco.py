#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

class Aruco:
  """Basic class to encapsulate the Aruco target tracking module in cv2"""

  _DICTS = {
    "4x4_100" : cv2.aruco.DICT_4X4_100,
    "4x4_1000" : cv2.aruco.DICT_4X4_1000,
    "4x4_250" : cv2.aruco.DICT_4X4_250,
    "4x4_50" : cv2.aruco.DICT_4X4_50,
    "5x5_100" : cv2.aruco.DICT_5X5_100,
    "5x5_1000" : cv2.aruco.DICT_5X5_1000,
    "5x5_250" : cv2.aruco.DICT_5X5_250,
    "5x5_50" : cv2.aruco.DICT_5X5_50,
    "6x6_100" : cv2.aruco.DICT_6X6_100,
    "6x6_1000" : cv2.aruco.DICT_6X6_1000,
    "6x6_250" : cv2.aruco.DICT_6X6_250,
    "6x6_50" : cv2.aruco.DICT_6X6_50,
    "7x7_100" : cv2.aruco.DICT_7X7_100,
    "7x7_1000" : cv2.aruco.DICT_7X7_1000,
    "7x7_250": cv2.aruco.DICT_7X7_250,
    "7x7_50": cv2.aruco.DICT_7X7_50,
    "apriltag_16h5" : cv2.aruco.DICT_APRILTAG_16H5,
    "apriltag_25h9" : cv2.aruco.DICT_APRILTAG_25H9,
    "apriltag_36h10" : cv2.aruco.DICT_APRILTAG_36H10,
    "apriltag_36h11" : cv2.aruco.DICT_APRILTAG_36H11,
    "aruco_original" : cv2.aruco.DICT_ARUCO_ORIGINAL
  }

  def __init__(self, image_topic, info_topic, dict_id, marker_length):
    self._bridge = cv_bridge.CvBridge()
    self._image_sub = rospy.Subscriber(image_topic, Image, self._image_callback)
    self._info_sub = rospy.Subscriber(info_topic, CameraInfo, self._info_callback)
    self._marker_length = marker_length
    dict = self._DICTS.get(dict_id.lower(), None)
    if dict == None:
      rospy.logerr(f"Aruco tag set {dict} not found")
    else:
      self._aruco_dict = cv2.aruco.Dictionary_get(dict)
      self._aruco_param = cv2.aruco.DetectorParameters_create()
      self._image = None
      self._cameraMatrix = None
      rospy.loginfo(f"using dictionary {dict_id}")

  def _display_tag(self, tag_id, image_size=256, save_to=None):
    tag = np.zeros([image_size, image_size, 1], dtype=np.uint8)
    cv2.aruco.drawMarker(self._aruco_dict, tag_id, image_size, tag, 1)
    rospy.loginfo(f"Writing tag {tag_id} to {save_to}")
    cv2.imwrite(f"{save_to}", tag)

  def _info_callback(self, msg):
    if msg.distortion_model != "plumb_bob":
      rospy.logerr(f"We can only deal with plumb_bob distortion {msg.distortion_model}")
    self._distortion = np.reshape(msg.D, (1,5))
    self._cameraMatrix = np.reshape(msg.K, (3,3))

  def _image_callback(self, msg):
    self._image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
    grey = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(grey, self._aruco_dict)
    frame = cv2.aruco.drawDetectedMarkers(self._image, corners, ids)
    if ids is None:
      rospy.loginfo(f"No targets found!")
      return
    if self._cameraMatrix is None:
      rospy.loginfo(f"We have not yet received a camera_info message")
      return

    rvec, tvec, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_length, self._cameraMatrix, self._distortion)
    result = self._image.copy()
    for r,t in zip(rvec,tvec):
      print(r)
      print(t)
      result = cv2.aruco.drawAxis(result, self._cameraMatrix, self._distortion, r, t, self._marker_length)
    cv2.imshow('window', result)
    cv2.waitKey(3)


if __name__ == '__main__':
  rospy.init_node('aruco_tracker')
  image_topic = rospy.get_param("~image", "/mycamera/image_raw")
  info_topic = rospy.get_param("~image", "/mycamera/camera_info")
  tag_set = rospy.get_param("~tags", "apriltag_36h10")
  marker_length = float(rospy.get_param("~width", "0.20"))
  write_markers = rospy.get_param("~write_markers", "False") == "True"

  x = Aruco(image_topic, info_topic, tag_set, marker_length)
  if write_markers:
    for id in range(10):
      x.display_tag(id, save_to=f"{tag_set}_{id}.png")
    rospy.loginfo("tags displayed")
  rospy.spin()
