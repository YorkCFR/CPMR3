#
# This network is based on the Line Follower Robot using CNN by Nawaz Ahmad
# towardsdatascience.com
#
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable information messages
from keras.models import Sequential
from keras.layers.core import Activation, Dense, Dropout
from keras import backend as K
from sklearn.model_selection import train_test_split
from keras.callbacks import TensorBoard
from tensorflow.keras.optimizers import Adam
import numpy as np
import random
from datetime import datetime

class LineFollower:
  def build():
    # initialize the model
    model = Sequential()
    model.add(Dense(8, input_shape=(2,), activation='relu'))
    model.add(Dense(8, activation='relu'))
    model.add(Dense(3, activation='softmax'))
    return model

dataset = np.loadtxt('database.csv', delimiter=',') # left-val right-val left centre right
X = dataset[:,0:2]
Y = dataset[:,2:5]
(trainX, testX, trainY, testY) = train_test_split(X, Y, shuffle=True, test_size=0.25, random_state=42)

logdir = "logs/scalars/" + datetime.now().strftime("%Y%m%d-%H%M%S")
tensorboard_callback = TensorBoard(log_dir=logdir)


# initialize the number of epochs to train for, initial learning rate,
# and batch size
EPOCHS = 200
INIT_LR = 1e-3
BS = 64
# initialize the model
print("[INFO] compiling model...")
model = LineFollower.build()
opt = Adam(lr=INIT_LR, decay=INIT_LR / EPOCHS)
model.compile(loss="binary_crossentropy", optimizer=opt, metrics=["accuracy"])
model.summary()
 
# train the network
print("[INFO] training network...")
H = model.fit(trainX, trainY, batch_size=BS,
    validation_data=(testX, testY),# steps_per_epoch=len(trainX) // BS,
    epochs=EPOCHS, verbose=1,
    callbacks=[tensorboard_callback])
 
# save the model to disk
print("[INFO] serializing network...")
model.save("line-follower")


