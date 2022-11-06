import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
import cv2
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class BuildMap(Node):
    _MAXD = 10
    _WIDTH = 513
    _HEIGHT = 513
    _M_PER_PIXEL = 0.02
    _ORIGIN_X = int(_WIDTH/2)
    _ORIGIN_Y = int(_HEIGHT/2)
    _MAP_NAME = "globalmap.pgm"

    def __init__(self):
        super().__init__('build_map')
        self.get_logger().info(f'{self.get_name()} created')

        self._map = np.zeros((BuildMap._HEIGHT, BuildMap._WIDTH), dtype=np.uint8)


        self.create_subscription(Odometry, "/odom", self._odom_callback, 1)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 1)

        self._cur_x = None
        self._cur_y = None
        self._cur_t = None
        self._map = np.full((BuildMap._HEIGHT, BuildMap._WIDTH), 255, np.uint8)


    def _scan_callback(self, msg):
        if self._cur_x is None:
            self.get_logger().info(f'{self.get_name()} ignoring scan until pose')
            return

        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        for i, r in enumerate(ranges):
            t = self._cur_t + angle_min + i * angle_increment 
            if not math.isinf(r):
                row = int(0.5 + BuildMap._ORIGIN_Y - (self._cur_y + r * math.sin(t)) / BuildMap._M_PER_PIXEL)
                col = int(0.5 + BuildMap._ORIGIN_X + (self._cur_x + r * math.cos(t)) / BuildMap._M_PER_PIXEL)

                if (row >= 0) and (col >= 0) and (row < BuildMap._HEIGHT) and (col < BuildMap._WIDTH):
                    self._map[row, col] = 0

        cv2.imshow('map',self._map)
        cv2.waitKey(10)

    def _odom_callback(self, msg):
        pose = msg.pose.pose

        o = pose.orientation
        roll, pitchc, yaw = euler_from_quaternion(o)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_t = yaw
        

def main(args=None):
    rclpy.init(args=args)
    node = BuildMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f'{node.get_name()} Writing map as {BuildMap._MAP_NAME}')
        cv2.imwrite(BuildMap._MAP_NAME, node._map)
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

