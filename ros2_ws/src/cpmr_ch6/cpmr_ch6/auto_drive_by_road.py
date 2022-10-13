import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable information messages
from packaging import version
import tensorflow as tf
if version.parse(tf.__version__) < version.parse("2.9.0"):
    from keras.preprocessing.image import img_to_array
else:
    from tensorflow.keras.utils import img_to_array
from keras.models import load_model
import math
import numpy as np
from keras.models import load_model
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class AutoDriveByLine(Node):
    def __init__(self):
        super().__init__('drive_by_line')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('image', "/mycamera/image_raw")
        self.declare_parameter('cmd', "/cmd_vel")
        self.declare_parameter('odom', "/odom")
        self.declare_parameter('rate', 10)
        self.declare_parameter('model', "road-follower")
        self.declare_parameter('x_vel', 0.2)
        self.declare_parameter('theta_vel', 0.2)
        self.declare_parameter('image_size', 28)


        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._cmd_topic = self.get_parameter('cmd').get_parameter_value().string_value
        self._rate = self.get_parameter('rate').get_parameter_value().double_value
        self._model = load_model(self.get_parameter('model').get_parameter_value().string_value)
        self._x_vel = self.get_parameter('x_vel').get_parameter_value().double_value
        self._theta_vel = self.get_parameter('theta_vel').get_parameter_value().double_value
        self._image_size = self.get_parameter('image_size').get_parameter_value().integer_value

        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self._pub = self.create_publisher(Twist, self._cmd_topic, 1)

        self._bridge = CvBridge()
        self._auto_driving = False

    def _image_callback(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
        cv2.imshow('window', image)
        key = cv2.waitKey(3)

        if self._auto_driving:
            if key == 125:
                self.get_logger().info(f"Auto driving ending")
                self._auto_driving = False
            image = cv2.resize(image, (self._image_size, self._image_size))
            im = img_to_array(image)
            im = np.array(im, dtype="float") / 255.0
            im = im.reshape(-1, self._image_size, self._image_size, 3)
            id = np.argmax(self._model.predict(im))
            if id == 0:
                self.turn_left()
            elif id == 1:
                self.go_straight()
            else:
                self.turn_right()
        else:
            if key == 106:
                self.turn_left()
            elif key == 107:
                self.go_straight()
            elif key == 108:
                self.turn_right()
            elif key == 32:
                self.stop()
            elif key == 113:
                self.get_logger().info(f"Closing node")
                exit(0)
            elif key == 120:
                self.get_logger().info(f"Auto driving starting")
                self._auto_driving = True
        

    def _command(self, x_vel, theta_vel):
        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = theta_vel
        self._pub.publish(twist)

    def go_straight(self):
        self._command(self._x_vel, 0.0)

    def turn_left(self):
        self._command(self._x_vel, self._theta_vel)

    def turn_right(self):
        self._command(self._x_vel, -self._theta_vel)

    def stop(self):
        self._command(0.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveByLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

