#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import CPUTemperature
from example_interfaces.msg import Float64

class TemperatureSensorNode(Node):
    def __init__(self):
        super().__init__('temperature_sensor')
        self.temperature_publisher_ = self.create_publisher(
            Float64, 'rpi_temperature', 10)
        self.temperature_timer_ = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info('Temperature Sensor Node has been started.')
       
    def publish_temperature(self):
        msg = Float64()
        msg.data = CPUTemperature().temperature
        self.temperature_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()
