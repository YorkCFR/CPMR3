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
from gazebo_msgs.srv import SpawnEntity, DeleteEntity



def make_marker(node, id, x0, y0, h, r):
   CYLINDER_MODEL = """
       <sdf version="1.6"> 				\
         <world name="default">                         \
           <model name="obstacle"> 			\
             <static>true</static> 			\
             <link name="all">                        	\
               <collision name="one">			\
                 <pose>0 0 {o} 0 0 0</pose>    		\
                 <geometry>				\
                   <cylinder>                       	\
                     <radius>0</radius>            	\
                     <length>0</length>            	\
                   </cylinder>   			\
                  </geometry>				\
               </collision>				\
               <visual name="two">			\
                 <pose>0 0 {o} 0 0 0</pose>    		\
                 <geometry>				\
                   <cylinder>                           \
                     <radius>{r}</radius>               \
                     <length>{h}</length>               \
                   </cylinder>                          \
                 </geometry>				\
               </visual>				\
             </link>                                    \
           </model>					\
         </world>                                       \
       </sdf>"""

   client = node.create_client(SpawnEntity, "/spawn_entity")
   node.get_logger().info("Connecting to /spawn_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = SpawnEntity.Request()
   request.name = id
   request.initial_pose.position.x = float(x0)
   request.initial_pose.position.y = float(y0)
   request.initial_pose.position.z = float(0)
   dict = {'h' : h, 'r':r, 'o': h/2}
   request.xml = CYLINDER_MODEL.format(**dict)
   node.get_logger().info(f"Making request...")
   client.call_async(request)
  

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
    AT_START = 'AT STart',
    HEADING_TO_TASK = 'Heading to Task',
    RETURNING_FROM_TASK = 'Returning from Task',
    TASK_DONE = 'Task Done'

class FSM(Node):

    def __init__(self):
        super().__init__('FSM')
        self.get_logger().info(f'{self.get_name()} created')

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_service(SetBool, '/startup', self._startup_callback)
        self.client = self.create_client(SpawnEntity, "/spawn_entity")
        self.get_logger().info("Connecting to /spawn_entity service...")
        self.client.wait_for_service()
        self.get_logger().info("Connected")
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_id = 0

        make_marker(self, "t"+str(self._last_id), self._last_x, self._last_y, 0.25, 0.01)

        # the blackboard
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_theta = 0.0
        self._cur_state = FSM_STATES.AT_START
        self._start_time = self.get_clock().now().nanoseconds * 1e-9
        self._points = [[0, 0, 0], [0, 5, 0], [5, 5, math.pi/2], [5, 0, -math.pi/2]]
        self._point = 0
        self._run = False

    def _startup_callback(self, request, resp):
        self.get_logger().info(f'Got a request {request}')
        if request.data:
            self.get_logger().info(f'fsm starting')
            self._run = True
            resp.success = True
            resp.message = "Architecture running"
        else:
            self.get_logger().info(f'fsm suspended')
            self._run = True
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
        
    def _drive_to_goal(self, goal_x, goal_y, goal_theta,
                       heading0_tol = 0.15,
                       range_tol = 0.15,
                       heading1_tol = 0.15):
        """Return True iff we are at the goal, otherwise drive there"""

        twist = Twist()

        goal_theta = FSM._short_angle(goal_theta) # goal in range -pi .. +pi

        x_diff = goal_x - self._cur_x
        y_diff = goal_y - self._cur_y
        dist = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        if dist > range_tol:
            self.get_logger().info(f'{self.get_name()} driving to goal with goal distance {dist}')
            # turn to the goal
            heading = math.atan2(y_diff, x_diff)
            diff = FSM._short_angle(heading - self._cur_theta)
            if (abs(diff) > heading0_tol):
                twist.angular.z = FSM._compute_speed(diff, 0.5, 0.2, 0.2)
                self.get_logger().info(f'{self.get_name()} turning towards goal heading {heading} current {self._cur_theta} diff {diff} {twist.angular.z}')
                self._publisher.publish(twist)
                self._cur_twist = twist
                return False

            twist.linear.x = FSM._compute_speed(dist, 0.5, 0.05, 0.5)
            self._publisher.publish(twist)
            self.get_logger().info(f'{self.get_name()} a distance {dist}  from target velocity {twist.linear.x}')
            self._cur_twist = twist
            return False

        # we are there, set the correct angle

        diff = FSM._short_angle(goal_theta - self._cur_theta)
        if abs(diff) > heading1_tol:
            twist.angular.z = FSM._compute_speed(diff, 0.5, 0.2, 0.2)
            self.get_logger().info(f'{self.get_name()} aligning with goal dst {dist} goal_theta {goal_theta} cur_theta {self._cur_theta} diff {diff} {twist.angular.z}')
            self._publisher.publish(twist)
            return False

        self.get_logger().info(f'at goal pose')
        self._publisher.publish(twist)
        return True


    def _do_state_at_start(self):
        self.get_logger().info(f'in start state')
        if self._run:
            self.get_logger().info(f'Starting...')
            self._cur_state = FSM_STATES.HEADING_TO_TASK

    def _do_state_heading_to_task(self):
        self.get_logger().info(f'heading to task {self._point}')
        if self._drive_to_goal(self._points[self._point][0], self._points[self._point][1], self._points[self._point][2]): 
            self._point = self._point + 1
            if self._point >= len(self._points):
                self._cur_state = FSM_STATES.RETURNING_FROM_TASK

    def _do_state_returning_from_task(self):
        self.get_logger().info(f'returning from task ')
        if self._drive_to_goal(0, 0, 0):
            self._publisher.publish(Twist())
            self._cur_state = FSM_STATES.TASK_DONE

    def _do_state_task_done(self):
        self.get_logger().info(f'{self.get_name()} task done')

    def _state_machine(self):
        if self._cur_state == FSM_STATES.AT_START:
            self._do_state_at_start()
        elif self._cur_state == FSM_STATES.HEADING_TO_TASK:
            self._do_state_heading_to_task()
        elif self._cur_state == FSM_STATES.RETURNING_FROM_TASK:
            self._do_state_returning_from_task()
        elif self._cur_state == FSM_STATES.TASK_DONE:
            self._do_state_task_done()
        else:
            self.get_logger().info(f'{self.get_name()} bad state {state_cur_state}')

    def _listener_callback(self, msg):
        pose = msg.pose.pose

        d2 = (pose.position.x - self._last_x) * (pose.position.x - self._last_x) + (pose.position.y - self._last_y) * (pose.position.y - self._last_y)
        if d2 > 0.5 * 0.5:
             self._last_id = self._last_id + 1
             self._last_x = pose.position.x
             self._last_y = pose.position.y
             make_marker(self, "t"+str(self._last_id), self._last_x, self._last_y, 0.25, 0.01)

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        self._cur_x = pose.position.x
        self._cur_y = pose.position.y
        self._cur_theta = FSM._short_angle(yaw)
        self._state_machine()



def main(args=None):
    rclpy.init(args=args)
    node = FSM()
    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

