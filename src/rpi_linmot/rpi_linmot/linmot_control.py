#! /usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from example_interfaces.msg import Float64
from enum import IntEnum

class LinmotCommand(IntEnum):
    """Command choices matching Arduino code"""
    OPERATIONAL_MODE = 1
    HOMING = 2
    GOTO_POSITION = 3
    SWITCH_ON = 4
    SWITCH_OFF = 5
    GET_POSITION = 6
    GOTO_POS_RESET_FORCE = 7
    SINE_MOTION = 8
    OPERATIONAL_MODE_ALT = 9
    CHANGE_TARGET_FORCE = 10
    GOTO_POS_WITH_FORCE = 11

class LinmotControlNode(Node): 
    def __init__(self,ser):
        super().__init__('linmot_control') 
        self.declare_parameter('control_method', 'joystick')
        self.ser_ = ser
        self.index_ = 0
        self.going_up_ = True
        self.control_method_ = self.get_parameter('control_method').value

        if self.control_method_ == "joystick":
            self.joy_sub_ = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        elif self.control_method_ == "automatic_sweep":
            self.auto_sweep_timer_ = self.create_timer(0.01, self.auto_sweep)
        elif self.control_method_ == "sinusoidal_sweep":
            self.sinusoidal_sweep_timer_ = self.create_timer(0.01, self.sinusoidal_sweep)

    def auto_sweep(self):
        cmd = "servo:" + str(self.index_) + "\n"
        self.ser_.write(cmd.encode('utf-8'))
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

    def joy_callback(self, msg):
        data = msg.axes[1]  
        # [-1,0,1.0]->[0,1.0]
        servo_position = int((data + 1.0) * 90)
        cmd  = "servo:" + str(servo_position) + "\n"
        self.ser_.write(cmd.encode('utf-8'))

    def sinusoidal_sweep(self):
        cmd  = "servo:" + str(servo_position) + "\n"
        self.ser_.write(cmd.encode('utf-8'))

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