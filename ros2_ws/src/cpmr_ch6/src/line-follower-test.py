import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable information messages
from keras.models import load_model
from imutils import paths
import numpy as np
import cv2
import random


model = load_model("line-follower")
dataset = np.loadtxt('database.csv', delimiter=',') # left-val right-val left centre right

res = ['left', 'forward', 'right']

vals=[[0,0,0],[0,0,0],[0,0,0]]


for item in dataset:
  im = item[0:2]
  im = np.array(im)
  im = im.reshape(-1,2)

  prediction = model.predict(im)
  user = np.argmax(item[2:5])
  id = np.argmax(prediction)
  vals[user][id] = vals[user][id] + 1
  print(vals)
  if user != id:
    print(f"{item[0]} {item[1]} {item[2]} {item[3]} {item[4]} user {res[user]} {prediction} {id} {res[id]}")
print(vals)


