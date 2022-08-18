#!/usr/bin/python3
#
# A very simple ros node to depopulate the world
#
import rospy
import os

def kill_can(id):
  os.system("rosservice call gazebo/delete_model '{model_name: can_" + str(id) + "}'")

if __name__ == '__main__':
  rospy.init_node('depopulate')
  
  for id in range(0,40):
    kill_can(id)
  rospy.signal_shutdown("all done")
    

