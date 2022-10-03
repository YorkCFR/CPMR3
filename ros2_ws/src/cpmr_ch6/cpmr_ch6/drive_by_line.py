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

class DriveByLine(Node):
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
        self.declare_parameter('output', "database.txt")
        self.declare_parameter('x_vel', 0.2)
        self.declare_parameter('theta_vel', 0.2)
        self.declare_parameter('height', 0.25)
        self.declare_parameter('width', 0.25)
        self.declare_parameter('separation', 0.50)
        self.declare_parameter('reflect', True)


        self._image_topic = self.get_parameter('image').get_parameter_value().string_value
        self._cmd_topic = self.get_parameter('cmd').get_parameter_value().string_value
        self._rate = self.get_parameter('rate').get_parameter_value().double_value
        self._output = self.get_parameter('output').get_parameter_value().string_value
        self._x_vel = self.get_parameter('x_vel').get_parameter_value().double_value
        self._theta_vel = self.get_parameter('theta_vel').get_parameter_value().double_value
        self._height = self.get_parameter('height').get_parameter_value().double_value
        self._width = self.get_parameter('width').get_parameter_value().double_value
        self._separation = self.get_parameter('separation').get_parameter_value().double_value
        self._reflect = self.get_parameter('reflect').get_parameter_value().bool_value

        self.create_subscription(Image, self._image_topic, self._image_callback, 1)
        self._pub = self.create_publisher(Twist, self._cmd_topic, 1)

        self._bridge = CvBridge()
        self._left_id = 0
        self._right_id = 0
        self._forward_id = 0
        self._recording = False
        self._curdir = DriveByLine._FORWARD
        self._fd = open(self._output,"w")
        self._left_val = 0
        self._right_val = 0

    def _image_callback(self, msg):
        image = self._bridge.imgmsg_to_cv2(msg, "bgr8") 
        if self._recording:
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
            self._right_val = np.mean(right) / 255.0
            left = image[r1:r2+1,c3:c4+1,:]
            self._left_val = np.mean(left) / 255.0
            self.get_logger().info(f"Sensors read {self._left_val} {self._right_val}")

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

        if key == 106:
            self.turn_left()
            self._curdir = DriveByLine._TURNING_LEFT
        elif key == 107:
            self.go_straight()
            self._curdir = DriveByLine._FORWARD
        elif key == 108:
            self.turn_right()
            self._curdir = DriveByLine._TURNING_RIGHT
        elif key == 32:
            self.stop()
            self._curdir = None
            self.get_logger().info(f"No recording when stopped")
        elif key == 115:
            self.get_logger().info(f"Start recording to {self._output} augmenting by reflection {self._reflect}")
            self._recording = True
        elif key == 120:
            self.get_logger().info(f"Stop recording left {self._left_id} forward {self._forward_id} right {self._right_id}")
            self._recording = False
        elif key == 113:
            self._fd.close()
            self.get_logger().info(f"Closing node")
            exit(0)
        

        if self._recording:
            if self._curdir == DriveByLine._FORWARD:
                self._fd.write(f"{self._left_val}, {self._right_val}, 0, 1, 0\n")
                self._forward_id = self._forward_id + 1
            elif self._curdir == DriveByLine._TURNING_LEFT:
                self._fd.write(f"{self._left_val}, {self._right_val}, 1, 0, 0\n")
                self._left_id = self._left_id + 1
                if self._reflect:
                    self._fd.write(f"{self._right_val}, {self._left_val}, 0, 0, 1\n")
                    self._right_id = self._right_id + 1
            elif self._curdir == DriveByLine._TURNING_RIGHT:
                self._fd.write(f"{self._left_val}, {self._right_val}, 0, 0, 1\n")
                self._right_id = self._right_id + 1
                if self._reflect:
                    self._fd.write(f"{self._right_val}, {self._left_val}, 1, 0, 0\n")
                    self._left_id = self._left_id + 1
      
      

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
    node = DriveByLine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

