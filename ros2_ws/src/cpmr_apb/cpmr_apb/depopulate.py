#
# A very simple ros node to depopulate the world
#
import os
import rclpy
from gazebo_msgs.srv import DeleteEntity

def remove_can(node, id):
   client = node.create_client(DeleteEntity, "/delete_entity")
   node.get_logger().info("Connecting to /delete_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = DeleteEntity.Request()
   request.name = f"can_{id}"
   node.get_logger().info("Making request...")
   future = client.call_async(request)
   rclpy.spin_until_future_complete(node, future)
   node.get_logger().info("...done")
   if not future.result().success:
       node.get_logger().info(f"Failure |{future.result()}|")
   

def main(args=None):
  rclpy.init(args=args)
  node = rclpy.create_node('depopulate')
  
  id = 0
  for id in range(0,40):
    remove_can(node, id)
  node.destroy_node()
  rclpy.shutdown()
    
if __name__ == '__main__':
    main()

