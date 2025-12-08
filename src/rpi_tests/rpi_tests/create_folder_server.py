#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import CreateFolder
import os

HOME = os.path.expanduser("~")

class CreateFolderServer(Node):
    def __init__(self):
        super().__init__('create_folder_server')
        self.create_folder_server_ = self.create_service(
            CreateFolder,
            'create_folder',
            self.create_folder_callback)
        self.get_logger().info('Create Folder Service has been started.')
        
    def create_folder_callback(self, request, response):
        folder_name = request.folder_name

        try:
            os.mkdir(HOME + "/" + folder_name)
            response.success = True
        except FileExistsError:
            response.success = False
            response.message = "Folder already exists."
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response
def main(args=None):
    rclpy.init(args=args)
    node = CreateFolderServer()
    rclpy.spin(node)
    rclpy.shutdown()