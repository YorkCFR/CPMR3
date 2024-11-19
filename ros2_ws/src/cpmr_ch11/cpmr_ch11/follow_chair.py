from enum import Enum
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool




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


class FSM_STATES(Enum):
    STARTUP = 'Waiting', 
    SLEEPING = 'Sleeping', 
    FOLLOWING = 'Following'

class FollowChair(Node):
    """This will make one chair (chair_name) move to whever another chair (target_name) 
       is right now. They will crash, of course, unless the chairs work to avoid this."""

    def __init__(self):
        super().__init__('FollowChair')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('chair_name', "chair_1")
        self._chair_name = self.get_parameter('chair_name').get_parameter_value().string_value
        self.declare_parameter('target_name', "chair_0")
        self._target_name = self.get_parameter('target_name').get_parameter_value().string_value
        self.get_logger().info(f'Chair {self._chair_name} is following {self._target_name}')

        self.create_subscription(Odometry, f"/{self._chair_name}/odom", self._self_callback, 1)
        self.create_subscription(Odometry, f"/{self._target_name}/odom", self._target_callback, 1)
        self._publisher = self.create_publisher(Twist, f"/{self._chair_name}/cmd_vel", 1)

        self.create_service(SetBool, f"/{self._chair_name}/startup", self._startup_callback)

        # the blackboard
        self._target_x = None
        self._target_y = None
        self._cur_state = FSM_STATES.STARTUP
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._run = False

    def _startup_callback(self, request, resp):
        self.get_logger().info(f'Got a request {request}')
        if request.data:
            resp.success = True
            resp.message = "Architecture running"
            self._cur_state = FSM_STATES.FOLLOWING
        else:
            if self._cur_state == FSM_STATES.STARTUP:
                self.get_logger().info(f'fsm suspended but not yet running?')
                resp.success = False
                resp.message = "In startup state"
            else:
                self._cur_state = FSM_STATES.SLEEPING
                self._publisher.publish(Twist())
                self.get_logger().info(f'fsm suspended')
                resp.success = True
                resp.message = "Architecture suspended"
        return resp
           

    def _short_angle(angle):
        if angle > math.pi:
            angle = angle - 2 * math.pi
        if angle < -math.pi:
            angle = angle + 2 * math.pi
        assert abs(angle) <= math.pi
        return angle

    def _compute_speed(diff, max_speed, min_speed, gain):
        speed = abs(diff) * gain
        speed = min(max_speed, max(min_speed, speed))
        return math.copysign(speed, diff)
        
    def _drive_to_target(self, heading0_tol = 0.15, range_tol = 1.0):
        """Return True iff we are at the goal, otherwise drive there. Goal in position space only"""

        twist = Twist()

        x_diff = self._target_x - self._cur_x
        y_diff = self._target_y - self._cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        if dist > range_tol:
            self.get_logger().info(f'{self.get_name()} driving to target with target distance {dist}')
            # turn to the goal
            heading = math.atan2(y_diff, x_diff)
            self.get_logger().info(f'Heading to target is {heading} cur_angle is {self._cur_theta}')
            diff = FollowChair._short_angle(heading - self._cur_theta)
            if (abs(diff) > heading0_tol):
                twist.angular.z = FollowChair._compute_speed(diff, 0.5, 0.2, 0.2)
                self.get_logger().info(f'{self.get_name()} turning towards goal heading {heading} current {self._cur_theta} diff {diff} {twist.angular.z}')
                self._publisher.publish(twist)
                self._cur_twist = twist
                return False

            twist.linear.x = FollowChair._compute_speed(dist, 0.5, 0.05, 0.5)
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} a distance {dist}  from target velocity {twist.linear.x}')
            self._cur_twist = twist
            return False

        self.get_logger().info(f'at target')
        self._publisher.publish(twist)
        return True


    def _do_state_at_start(self):
        self.get_logger().info(f'waiting in start state')
        pass

    def _do_state_following(self):
        self.get_logger().info(f'following the target')
        if self._target_x is not None:
            self._drive_to_target()
            self._target_x = None
            self._target_y = None

    def _state_machine(self):
        if self._cur_state == FSM_STATES.STARTUP:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.FOLLOWING:
            self._do_state_following()
        elif self._cur_state == FSM_STATES.SLEEPING:
            pass
        else:
            self.get_logger().info(f'Bad state {state_cur_state}')

    def _target_callback(self, msg):
        """Update from target received"""
        pose = msg.pose.pose
        self._target_x = pose.position.x
        self._target_y = pose.position.y

    def _self_callback(self, msg):
        """We got a pose update"""
        pose = msg.pose.pose

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = FollowChair._short_angle(yaw)
        self._state_machine()



def main(args=None):
    rclpy.init(args=args)
    node = FollowChair()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

