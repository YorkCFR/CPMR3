# A very simple ros node to test the arm
#
import time
import rclpy
import os
from kinova_interfaces.srv import GetGripper, SetGripper, SetConfiguration, Home


def home_robot(node):
    client = node.create_client(Home, "/home")
    client.wait_for_service()
    request = Home.Request()
    future = client.call_async(request)
    while not future.done():
        rclpy.spin_once(node)
    node.destroy_client(client)
    return future.result().status

def get_gripper(node):
    client = node.create_client(GetGripper, "/get_gripper")
    client.wait_for_service()
    request = GetGripper.Request()
    future = client.call_async(request)
    while not future.done():
        rclpy.spin_once(node)
    node.destroy_client(client)
    return future.result().gripper

def set_gripper(node, value):
    client = node.create_client(SetGripper, "/set_gripper")
    client.wait_for_service()
    request = SetGripper.Request()
    request.gripper = float(value)
    future = client.call_async(request)
    while not future.done():
        rclpy.spin_once(node)
    node.destroy_client(client)
    return future.result().status

def set_configuration(node, joints):
    client = node.create_client(SetConfiguration, "/set_configuration")
    client.wait_for_service()
    request = SetConfiguration.Request()
    request.joints = joints
    future = client.call_async(request)
    while not future.done():
        rclpy.spin_once(node)
    node.destroy_client(client)
    return future.results().status

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('arm_test')
    print(f"Homing robot {home_robot(node)}")
    print("Closing and opening gripper")
    set_gripper(node, 1.0)
    for i in range(10):
        print(get_gripper(node))
        time.sleep(0.2)
    set_gripper(node, 0)
    for i in range(10):
        print(get_gripper(node))
        time.sleep(0.2)
    print(f"set_configuration(node, [0,0,0,0,0,0])")
    print(f"set_configuration(node, [154,0,0,0,0,0])")
    print(f"set_configuration(node, [-154,0,0,0,0,0])")
    print(f"set_configuration(node, [0,0,0,0,0,0])")
    print(f"set_configuration(node, [0,-45,0,0,0,0])")
    print(f"set_configuration(node, [0,45,0,0,0,0])")
    print(f"set_configuration(node, [0,0,0,0,0,0])")
    print(f"set_configuration(node, [0,0,45,0,0,0])")
    print(f"set_configuration(node, [0,0,-45,0,0,0])")
    print(f"set_configuration(node, [0,0,0,0,0,0])")
    print(f"set_configuration(node, [0,0,0,45,0,0])")
    print(f"set_configuration(node, [0,0,0,-45,0,0])")
    print(f"set_configuration(node, [0,0,0,0,0,0])")
    print(f"set_configuration(node, [0,0,0,0,45,0])")
    print(f"set_configuration(node, [0,0,0,0,-45,0])")
    print(f"set_configuration(node, [0,0,0,0,0,0])")
    print(f"set_configuration(node, [0,0,0,0,0,45])")
    print(f"set_configuration(node, [0,0,0,0,0,-45])")
    print(f"set_configuration(node, [45,-45,0,0,0,0])")
    input("Press return to grab block")
    set_gripper(node, 1.0)
    time.sleep(2)
    print(get_gripper(node))
    print(f"set_configuration(node, [0,-45,45,0,0,0])")
    set_gripper(node, 0.0)
    time.sleep(2)
    print(f"Homing robot {home_robot(node)}")

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

