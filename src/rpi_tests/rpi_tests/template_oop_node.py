#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node): #Modify name
    def __init__(self):
        super().__init__('my_custom_node') #Modify name
        

def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode() #Modify name
    rclpy.spin(node)
    rclpy.shutdown()