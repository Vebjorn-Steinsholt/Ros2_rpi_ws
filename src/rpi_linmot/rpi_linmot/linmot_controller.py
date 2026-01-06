#!/usr/bin/env python3
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from example_interfaces.msg import String, Float64
from enum import Enum, auto

class StartupState(Enum):
    """Simple startup states"""
    INIT = auto()
    OPERATIONAL = auto()
    SWITCH_ON = auto()
    HOMING = auto()
    READY = auto()
    ERROR = auto()

class LinmotStartupNode(Node):
    def __init__(self, ser):
        super().__init__('linmot_startup')
        
        self.ser_ = ser
        self.state_ = StartupState.INIT
        self.status_word_ = 0
        self.current_position_ = 0.0
        
        # Publishers
        self.state_pub_ = self.create_publisher(String, 'linmot/state', 10)
        self.position_pub_ = self.create_publisher(Float64, 'linmot/position', 10)
        
        # Timers
        self.state_machine_timer_ = self.create_timer(0.5, self.state_machine_update)
        self.serial_read_timer_ = self.create_timer(0.05, self.read_serial)
        self.feedback_timer_ = self.create_timer(0.1, self.request_position)
        
        self.get_logger().info('Starting LinMot startup sequence...')
    
    def send_command(self, choice, target_pos=0, vel=0, acc=0, dec=0, force=0, limit=0):
        """Send CSV command to Arduino"""
        cmd = f"{choice},{target_pos},{vel},{acc},{dec},{force},{limit}\n"
        self.ser_.write(cmd.encode())
        self.get_logger().info(f"Sent: choice={choice}")
    
    def read_serial(self):
        """Read responses from Arduino"""
        while self.ser_.in_waiting > 0:
            line = self.ser_.readline().decode().strip()
            
            if line.startswith("Sending:"):
                # Parse: "Sending: <pos>,<status>"
                data = line.replace("Sending:", "").strip()
                parts = data.split(',')
                if len(parts) == 2:
                    self.current_position_ = float(parts[0])
                    self.status_word_ = int(parts[1])
                    
                    # Publish position
                    pos_msg = Float64()
                    pos_msg.data = self.current_position_
                    self.position_pub_.publish(pos_msg)
            else:
                self.get_logger().info(f"Arduino: {line}")
    
    def request_position(self):
        """Request position feedback"""
        if self.state_ != StartupState.INIT:
            self.send_command(6)  # GET_POSITION
    
    def is_homed(self):
        """Check if bit 11 (HOMED) is set"""
        return bool(self.status_word_ & (1 << 11))
    
    def is_operation_enabled(self):
        """Check if bit 0 (OPERATION_ENABLED) is set"""
        return bool(self.status_word_ & (1 << 0))
    
    def state_machine_update(self):
        """Main state machine - runs every 0.5 seconds"""
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.state_.name
        self.state_pub_.publish(state_msg)
        
        # State machine logic
        if self.state_ == StartupState.INIT:
            self.get_logger().info("STATE: INIT - Sending operational mode")
            self.send_command(1)  # OPERATIONAL_MODE
            time.sleep(0.3)
            self.state_ = StartupState.OPERATIONAL
        
        elif self.state_ == StartupState.OPERATIONAL:
            self.get_logger().info("STATE: OPERATIONAL - Switching on drive")
            self.send_command(4)  # SWITCH_ON
            time.sleep(0.3)
            self.state_ = StartupState.SWITCH_ON
        
        elif self.state_ == StartupState.SWITCH_ON:
            if self.is_operation_enabled():
                self.get_logger().info("STATE: SWITCH_ON - Operation enabled! Starting homing")
                self.send_command(2)  # HOMING
                self.state_ = StartupState.HOMING
            else:
                self.get_logger().info(f"Waiting for operation enabled... Status: 0x{self.status_word_:04X}")
        
        elif self.state_ == StartupState.HOMING:
            if self.is_homed():
                self.get_logger().info(f"STATE: HOMING COMPLETE! Position: {self.current_position_} mm")
                self.state_ = StartupState.READY
            else:
                self.get_logger().info(f"Homing... Position: {self.current_position_} mm, Status: 0x{self.status_word_:04X}")
        
        elif self.state_ == StartupState.READY:
            self.get_logger().info(f"STATE: READY - Motor ready at {self.current_position_} mm")
            # Stop the state machine timer once ready
            self.state_machine_timer_.cancel()
            self.get_logger().info("Startup complete! You can now send position commands.")
        
        elif self.state_ == StartupState.ERROR:
            self.get_logger().error(f"STATE: ERROR - Status: 0x{self.status_word_:04X}")

def main(args=None):
    # Connect to Arduino
    print("Connecting to Arduino...")
    while True:
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1.0)
            print("Serial port opened!")
            time.sleep(2)
            ser.reset_input_buffer()
            break
        except serial.SerialException as e:
            print(f"Error: {e}. Retrying in 1 second...")
            time.sleep(1)
    
    # Start ROS2 node
    rclpy.init(args=args)
    node = LinmotStartupNode(ser)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.send_command(5)  # SWITCH_OFF
        ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()