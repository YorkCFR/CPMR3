# A very simple ros node to populate the world
#
import rclpy
import os
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

def make_obstacle(node, id, x0, y0, h, r):
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
                     <radius>{r}</radius>            	\
                     <length>{h}</length>            	\
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
   future = client.call_async(request)
   while not future.done():
       rclpy.spin_once(node)
   node.get_logger().info("...done")
   if not future.result().success:
       node.get_logger().info(f"Failure {future.result()}")
  
def remove_obstacle(node, id):
   client = node.create_client(DeleteEntity, "/delete_entity")
   node.get_logger().info("Connecting to /delete_entity service...")
   client.wait_for_service()
   node.get_logger().info("...connected")
   request = DeleteEntity.Request()
   request.name = id
   node.get_logger().info("Making request...")
   future = client.call_async(request)
   rclpy.spin_until_future_complete(node, future)
   node.get_logger().info("...done")
   if not future.result().success:
       node.get_logger().info(f"Failure {future.result()}")



def main(args=None):
  rclpy.init(args=args)
  node = rclpy.create_node('demo')
  make_obstacle(node, 'blob', 2, 3, 2, 1)
  
  node.destroy_node()
  rclpy.shutdown()
    
if __name__ == '__main__':
    main()

