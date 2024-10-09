import cv2
import numpy as np

size = 512   # image will be this size/square
dict = cv2.aruco.DICT_APRILTAG_36h10    # dictionary to use
id = 4  # id within the dictionary
name = "tag.jpg"

dict = cv2.aruco.Dictionary_get(dict)

tag = np.zeros((size, size, 1), dtype="uint8")
cv2.aruco.drawMarker(dict, id, size, tag, 1)
cv2.imwrite(name, tag)

