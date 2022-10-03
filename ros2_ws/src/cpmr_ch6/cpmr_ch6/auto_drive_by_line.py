import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2' # disable information messages
from keras.preprocessing.image import img_to_array
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
        self.declare_parameter('model', "line-follower")
        self.declare_parameter('x_vel', 0.2)
        self.declare_parameter('theta_vel', 0.2)
        self.declare_parameter('height', 0.25)
        self.declare_parameter('width', 0.25)
        self.declare_parameter('separation', 0.50)
        self.declare_parameter('reflect', True)


        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._cmd_topic = self.get_parameter('cmd').get_parameter_value().string_value
        self._rate = self.get_parameter('rate').get_parameter_value().double_value
        self._model = load_model(self.get_parameter('model').get_parameter_value().string_value)
        self._x_vel = self.get_parameter('x_vel').get_parameter_value().double_value
        self._theta_vel = self.get_parameter('theta_vel').get_parameter_value().double_value
        self._height = self.get_parameter('height').get_parameter_value().double_value
        self._width = self.get_parameter('width').get_parameter_value().double_value
        self._separation = self.get_parameter('separation').get_parameter_value().double_value
        self._reflect = self.get_parameter('reflect').get_parameter_value().bool_value

        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self._pub = self.create_publisher(Twist, self._cmd_topic, 1)

        self._bridge = CvBridge()
        self._auto_driving = False

    def _image_callback(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
        w = image.shape[1]
        h = image.shape[0]
        w1 = int(w * self._width)
        h1 = int(h * self._height)
        ox = int(w * self._separation/2)
            
        c1 = int(w/2+ox/2)
        c2 = int(w/2+ox/2+w1)
        c3 = int(w/2-w1-ox/2)
        c4 = int(w/2-w1-ox/2+w1)
        r1 = int(h/2-h1/2)
        r2 = int(h/2+h1/2)

        right = image[r1:r2+1,c1:c2+1,:]
        right_val = np.mean(right) / 255.0
        left = image[r1:r2+1,c3:c4+1,:]
        left_val = np.mean(left) / 255.0
        self.get_logger().info(f"Sensors read {left_val} {right_val}")

        cv2.line(image,(c1, r1), (c2, r1), (0,0,255))
        cv2.line(image,(c1, r2), (c2, r2), (0,0,255))
        cv2.line(image,(c2, r1), (c2, r2), (0,0,255))
        cv2.line(image,(c1, r1), (c1, r2), (0,0,255))

        cv2.line(image,(c3, r1), (c4, r1), (0,0,255))
        cv2.line(image,(c3, r2), (c4, r2), (0,0,255))
        cv2.line(image,(c4, r1), (c4, r2), (0,0,255))
        cv2.line(image,(c3, r1), (c3, r2), (0,0,255))
        cv2.imshow('window', image)
        key = cv2.waitKey(3)

        if self._auto_driving:
            if key == 120:
                self.get_logger().info(f"Auto driving ending")
                self._auto_driving = False
            im = [left_val, right_val]
            im = np.array(im)
            im = im.reshape(-1,2)

            prediction = self._model.predict(im)
            id = np.argmax(prediction)
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

