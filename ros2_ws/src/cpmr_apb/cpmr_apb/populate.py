# A very simple ros node to populate the world
#
import rclpy
import os
from gazebo_msgs.srv import SpawnEntity

def make_can(node, id, x, y):
   COKE_CAN_MODEL = """\ 
       <sdf version="1.6">
           <world name="default">
               <include>
                   <uri>model://coke_can</uri>
               </include>
           </world>
       </sdf>"""

   client = node.create_client(SpawnEntity, "/spawn_entity")
   node.get_logger().info("Connecting to /spawn_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = SpawnENtity.Request()
   request.name = f"can_{id}"
   request.initial_pose.position.x = x
   request.initial_pose.position.y = y
   request.initial_pose.position.z = 0
   request.xml = COKE_CAN_MODEL
   node.get_logger().info("Making request...")
   future = client.call_async(request)
   rclpy.spin_until_future_complete(node, future)
   node.get_logger().info("...done")
   if future.success is not None:
       node.get_logger().info(f"Failure |{future.status_message}|")
  


def main(args=None):
  rclpy.init(args=args)
  node = rclpy.create_node('populate')
  
  id = 0
  for z in range(0,10):
    make_can(node, id, -1, -1 + z / 5)
    id = id + 1
    make_can(id, -1 + z / 5, 1)
    id = id + 1
    make_can(id, 1, 1 - z / 5)
    id = id + 1
    make_can(id, 1 - z / 5, -1)
    id = id + 1
  node.destroy_node()
  rclpy.shutdown()
    
if __name__ == '__main__':
    main()

