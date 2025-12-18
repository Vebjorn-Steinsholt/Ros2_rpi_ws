#! /usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from sensor_msgs.msg import Joy



class ArduinoDriverNode(Node): 
    def __init__(self, ser):
        super().__init__('arduino_driver')
        self.ser_ = ser
        self.led_on_ = False
        self.temperature_publisher_ = self.create_publisher(Int64, 'arduino_temperature', 10)
        self.blink_led_timer_ = self.create_timer(1.0, self.blink_led)
        self.read_serial_timer_ = self.create_timer(0.01, self.read_serial)
        
        self.get_logger().info("Arduino Driver Node has been started.")

    def blink_led(self):
        cmd = "led:" + str(int(self.led_on_))+"\n"
        self.ser_.write(cmd.encode('utf-8'))
        self.led_on_ = not self.led_on_

    def read_serial(self):
        if self.ser_.in_waiting > 0:
            data = self.ser_.readline().decode('utf-8').rstrip()
            # Accept only numeric payloads; skip status lines like "OK:" or "ERR:".
            if data and not data.startswith("OK:") and not data.startswith("ERR:"):
                try:
                    value = int(data)
                except ValueError:
                    # Non-numeric line; ignore it quietly.
                    return
                msg = Int64()
                msg.data = value
                self.temperature_publisher_.publish(msg)


def main(args=None):
    # Init Serial Communication
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
            print("Serial port opened:", ser.is_open)
            time.sleep(2)  # Give device time to stabilize
            ser.reset_input_buffer()
            print("Input buffer reset successfully")
            break
        except serial.SerialException:
            print("Error opening serial port, retrying in 1 second...")
            time.sleep(1)
    rclpy.init(args=args)
    node = ArduinoDriverNode(ser)
    rclpy.spin(node)
    ser.close()
    rclpy.shutdown()