#
import sys
import rclpy
from rclpy.node import Node
from kinova_interfaces.srv import GetGripper, SetGripper, SetConfiguration, Home


class KinovaGen3(Node):

    def __init__(self):
        super().__init__('kinova_gen3')
        self.get_logger().info(f'{self.get_name()} starting')

        self.declare_parameter('simulate', True)
        self._simulate = self.get_parameter('simulate').get_parameter_value().bool_value
        self.get_logger().info(f'{self.get_name()} running with simulate {self._simulate}')

        self.create_service(Home, "home", self._home)
        self.create_service(GetGripper, "get_gripper", self._get_gripper)
        self.create_service(SetGripper, "set_gripper", self._set_gripper)
        self.create_service(SetConfiguration, "set_configuration", self._set_configuration)

        if self._simulate:
            self._base = None
        else:
            from kinova_gen3.gen3_glue import set_gripper, get_gripper, move_to_home_position, angular_action_movement, connect
            self._base = connect()

    def _home(self, request, response):
        self.get_logger().info(f'{self.get_name()}  moving to home')
        if self._simulate:
            response.status = True
        else:
            response.status = move_to_home_position(self._base)
        return response

    def _get_gripper(self, request, response):
        self.get_logger().info(f'{self.get_name()}  get_gripper: callback {request}')
        if self._simulate:
            response.gripper = 1.23
        else:
            response.gripper = get_gripper(self._base)
        return response

    def _set_gripper(self, request, response):
        self.get_logger().info(f'{self.get_name()}  set_gripper: callback {request.gripper}')
        if self._simulate:
           response.status = True
        else:
           response.status = set_gripper(self._base, request.gripper)
        return response

    def _set_configuration(self, request, response):
        self.get_logger().info(f'{self.get_name()}  set_configuration : callback {request.joints}')
        if self._simulate:
            response.status = True
        else:
            response.status = anglular_action_movement(self._base, request.joints) 
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KinovaGen3()
    try:
       rclpy.spin(node)
    except  KeyboardInterrupt:
       pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
