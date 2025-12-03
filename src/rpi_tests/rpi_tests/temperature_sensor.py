#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import CPUTemperature

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.get_logger().info("%lf" % (CPUTemperature().temperature,))
        self.get_logger().info('Temperature Sensor Node has been started.')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
