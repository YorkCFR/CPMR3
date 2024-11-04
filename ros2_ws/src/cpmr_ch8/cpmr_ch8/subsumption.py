import math
import random
import numpy as np
import rclpy
import cv2
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge, CvBridgeError

class Subsumption(Node):

    def _wander(self, max_dtheta = math.pi / 20, max_thetav = math.pi / 10, max_dv = .5, max_v = 1.2):
        """wander randomly"""
        twist = Twist()
        v = self._last_twist.linear.x + (2 * random.random() - 1) * max_dv
        t = self._last_twist.angular.z + (2 * random.random() -  1) * max_dtheta
   
        twist.linear.x = max(min(v, max_v), 0.0)
        twist.angular.z = max(min(t, max_thetav), 0.0)

        return twist

    def _avoid_obstacle(self, minr = 1.5):
        """ if there is an obstacle within mind of the front of the robot, stop and rotate"""
        if self._min_r < minr:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = math.pi / 10
            return twist
        return None

    def _get_red(self, count = 2000):
        if self._redcolcount > count:
            twist = Twist()
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            return twist
        return None
        
    
    def __init__(self):
        super().__init__('subsumption')
        self.get_logger().info(f'{self.get_name()} created')

        self.create_subscription(Image, '/mycamera/image_raw', self._image_callback, 1)
        self._bridge = CvBridge()
        self.create_subscription(LaserScan, "/scan", self._laser_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_timer(0.01, self._timer_callback)
        self.create_service(SetBool, '/startup', self._startup_callback)

        self._run = False
        self._last_twist = Twist()
        self._min_r = 10000
        self._redcolcount = 0

    def _startup_callback(self, request, resp):
        self.get_logger().info(f'Got a request {request}')
        if request.data:
            self.get_logger().info(f'subsumption starting')
            self._run = True
            resp.success = True
            resp.message = "Architecture running"
        else:
            self.get_logger().info(f'subsumption suspended')
            self._run = True
            resp.success = True
            resp.message = "Architecture suspended"
        return resp
           

    def _timer_callback(self):
        if not self._run:
            return
        now = self.get_clock().now().nanoseconds
        wander = self._wander()
        avoid = self._avoid_obstacle()
        red = self._get_red()
        if avoid is not None:
            self.get_logger().info(f'{now}, avoiding')
            self._publisher.publish(avoid)
            return
        if red is not None:
            self.get_logger().info(f'{now}, going for red')
            self._publisher.publish(red)
            return
        self.get_logger().info(f'{now}, wandering')
        self._publisher.publish(wander)

    def _laser_callback(self, msg, mind=1.5):
        min_range = mind * 10
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if (abs(angle) < math.pi/4) and (r < min_range):
                min_range = r
        self._min_r = min_range


    def _image_callback(self, msg, width= 25):
        """ lots of red? drive to it"""
        image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        redcol = 0
        redcolcount = 0
        for i in range(image.shape[0]):
            for j in range(int(image.shape[1]/2-width), int(image.shape[1]/2+width)):
                if (image[i][j][2] > 100) and (image[i][j][0] < 50):
                    redcolcount = redcolcount + 1
        self._redcolcount =  redcolcount



def main(args=None):
    rclpy.init(args=args)
    node = Subsumption()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

