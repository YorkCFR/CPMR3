# A very simple ros node to populate the world
#
import rclpy
import os
from gazebo_msgs.srv import SpawnEntity

def make_can(node, id, x, y):
   COKE_CAN_MODEL = """ <sdf version="1.6"> <world name="default"> <include> <uri>model://coke_can</uri> </include> </world> </sdf>"""

   client = node.create_client(SpawnEntity, "/spawn_entity")
   node.get_logger().info("Connecting to /spawn_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = SpawnEntity.Request()
   request.name = f"can_{id}"
   request.initial_pose.position.x = float(x)
   request.initial_pose.position.y = float(y)
   request.initial_pose.position.z = float(0)
   request.xml = COKE_CAN_MODEL
   node.get_logger().info("fMaking request...{COKE_CAN_MODEL}")
   future = client.call_async(request)
   while not future.done():
       rclpy.spin_once(node)
   node.get_logger().info("...done")
   node.get_logger().info(f"{future.result()}")
  


def main(args=None):
  rclpy.init(args=args)
  node = rclpy.create_node('populate')
  
  id = 0
  for z in range(0,10):
    make_can(node, id, -1, -1 + z / 5)
    id = id + 1
    make_can(node, id, -1 + z / 5, 1)
    id = id + 1
    make_can(node, id, 1, 1 - z / 5)
    id = id + 1
    make_can(node, id, 1 - z / 5, -1)
    id = id + 1
  node.destroy_node()
  rclpy.shutdown()
    
if __name__ == '__main__':
    main()

