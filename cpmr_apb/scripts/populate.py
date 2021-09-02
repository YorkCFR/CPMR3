#!/usr/bin/python3
#
# A very simple ros node to populate the world
#
import rospy
import os

def make_can(id, x, y):
  os.system(f"rosrun gazebo_ros spawn_model -database coke_can -sdf -model can_{id} -x {x} -y {y}")

if __name__ == '__main__':
  rospy.init_node('populate')
  
  id = 0
  for z in range(0,10):
    make_can(id, -1, -1 + z / 5)
    id = id + 1
    make_can(id, -1 + z / 5, 1)
    id = id + 1
    make_can(id, 1, 1 - z / 5)
    id = id + 1
    make_can(id, 1 - z / 5, -1)
    id = id + 1
  rospy.signal_shutdown("all done")
    

