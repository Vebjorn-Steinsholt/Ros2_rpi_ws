#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class CreateFolderServer(Node):
    def __init__(self):
        super().__init__('create_folder_server')
        

def main(args=None):
    rclpy.init(args=args)
    node = CreateFolderServer()
    rclpy.spin(node)
    rclpy.shutdown()