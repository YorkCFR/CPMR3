# write to output/X/  where X is one of forward, left, right
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class DriveByRoad(Node):
    _FORWARD = 1
    _TURNING_LEFT = 2
    _TURNING_RIGHT = 3
  
    def __init__(self):
        super().__init__('drive_by_line')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('image', "/mycamera/image_raw")
        self.declare_parameter('cmd', "/cmd_vel")
        self.declare_parameter('odom', "/odom")
        self.declare_parameter('rate', 10)
        self.declare_parameter('output', "output")
        self.declare_parameter('x_vel', 0.2)
        self.declare_parameter('theta_vel', 0.2)
        self.declare_parameter('image_size', 28)


        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._cmd_topic = self.get_parameter('cmd').get_parameter_value().string_value
        self._rate = self.get_parameter('rate').get_parameter_value().double_value
        self._output = self.get_parameter('output').get_parameter_value().string_value
        self._x_vel = self.get_parameter('x_vel').get_parameter_value().double_value
        self._theta_vel = self.get_parameter('theta_vel').get_parameter_value().double_value
        self._image_size = self.get_parameter('image_size').get_parameter_value().integer_value

        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self._pub = self.create_publisher(Twist, self._cmd_topic, 1)

        self._bridge = CvBridge()
        self._recording = False
        self._left_id = 0
        self._right_id = 0
        self._forward_id = 0
        self._curdir = None

    def _image_callback(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
        cv2.imshow('window', image)
        key = cv2.waitKey(3)

        if key == 106:
            self.turn_left()
            self._curdir = DriveByRoad._TURNING_LEFT
        elif key == 107:
            self.go_straight()
            self._curdir = DriveByRoad._FORWARD
        elif key == 108:
            self.turn_right()
            self._curdir = DriveByRoad._TURNING_RIGHT
        elif key == 32:
            self.stop()
            self._curdir = None
            self._recording = False
            print(f"No recording when stopped")
        elif key == 115:
            self.get_logger().info(f"Start recording to {self._output}/X/ where X is one of forward, left, right")
            self._recording = True
            self._curdir = DriveByRoad._FORWARD
            self.go_straight()
        elif key == 120:
            self.get_logger().info(f"Stop recording left {self._left_id} forward {self._forward_id} right {self._right_id}")
            self._recording = False

        if self._recording:
            dim = min(image.shape[0], image.shape[1])
            oy = int((image.shape[0] - dim) / 2)
            ox = int((image.shape[1] - dim) / 2)
            roi = image[oy:oy+dim, ox:ox+dim, 0:3]
            out = cv2.resize(image, (self._image_size, self._image_size))
            cv2.imshow("Resized", out)
            if self._curdir == DriveByRoad._FORWARD:
                cv2.imwrite(f"{self._output}/forward/image.{self._forward_id:08d}.jpg", out)
                self._forward_id = self._forward_id + 1
            elif self._curdir == DriveByRoad._TURNING_LEFT:
                cv2.imwrite(f"{self._output}/left/image.{self._left_id:08d}.jpg", out)
                self._left_id = self._left_id + 1
            elif self._curdir == DriveByRoad._TURNING_RIGHT:
                cv2.imwrite(f"{self._output}/right/image.{self._right_id:08d}.jpg", out)
                self._right_id = self._right_id + 1
        cv2.imshow('window', image)

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
    node = DriveByRoad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

