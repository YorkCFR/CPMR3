from packaging import version
import tensorflow as tf
if version.parse(tf.__version__) < version.parse("2.9.0"):
    from keras.preprocessing.image import img_to_array
else:
    from tensorflow.keras.utils import img_to_array
from keras.models import load_model
from imutils import paths
import numpy as np
import cv2
import random


model = load_model("road-follower")
dataset = './trainImages/' 
imagePaths = sorted(list(paths.list_images(dataset)))

dirs = ["left", "forward", "right"]
for name in imagePaths:
  image = cv2.imread(name)
  cv2.imshow("Image", image)
  cv2.waitKey(3)
  im = img_to_array(image)
  im = np.array(im, dtype="float") / 255.0
  im = im.reshape(-1, 28, 28, 3)
  prediction = np.argmax(model.predict(im))
  if name.find(dirs[prediction]) < 0:
    print(f"Failure {name} direction is {dirs[prediction]}")

