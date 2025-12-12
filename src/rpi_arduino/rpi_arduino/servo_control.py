#! /usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node

class ServoControlNode(Node): 
    def __init__(self,ser):
        super().__init__('servo_control') 
        self.ser_ = ser
        self.index_ = 0
        self.going_up_ = True
        self.servo_sweep_timer_ = self.create_timer(0.01, self.servo_sweep)

    def servo_sweep(self):
        print(self.index_)
        if(self.going_up_):
            if self.index_ >= 180:
                self.going_up_ = False
            else:
                self.index_+=1
        else:
            if self.index_ <= 0:
                self.going_up_ = True
            else:
                self.index_ -= 1
        self.ser_.write("servo:"+index

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
    node = ServoControlNode(ser) 
    rclpy.spin(node)
    ser.close()
    rclpy.shutdown()