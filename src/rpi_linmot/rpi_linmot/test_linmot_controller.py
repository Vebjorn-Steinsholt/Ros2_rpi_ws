#!/usr/bin/env python3
"""
Test script for Arduino Linmot Serial Controller
Tests serial communication and various control commands
Run on Raspberry Pi connected to Arduino
"""

import time
import serial
import sys
import argparse
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

class LinmotTester:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200, timeout=2.0):
        """Initialize serial connection to Arduino"""
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connect()
        
    def connect(self):
        """Establish serial connection with retry logic"""
        max_retries = 5
        retry_count = 0
        
        while retry_count < max_retries:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                print(f"✓ Serial port opened: {self.ser.is_open}")
                time.sleep(2)  # Allow Arduino to reset
                self.ser.reset_input_buffer()
                print("✓ Input buffer reset")
                
                # Wait for Arduino setup confirmation
                start_time = time.time()
                while time.time() - start_time < 5:
                    if self.ser. in_waiting > 0:
                        line = self.ser. readline().decode('utf-8', errors='ignore').strip()
                        print(f"Arduino:  {line}")
                        if "SETUP.. OK" in line:
                            print("✓ Arduino ready")
                            return True
                return True
                
            except serial.SerialException as e:
                retry_count += 1
                print(f"✗ Error opening serial port (attempt {retry_count}/{max_retries}): {e}")
                time.sleep(1)
        
        print("✗ Failed to establish serial connection")
        sys.exit(1)
    
    def send_command(self, choice, target_pos=0.0, max_vel=0.0, acc=0.0, dec=0.0, 
                     target_force=0.0, force_limit=0.0):
        """
        Send command to Arduino in CSV format
        
        Args:
            choice: Command choice (1-11)
            target_pos: Target position in mm
            max_vel: Maximum velocity in m/s
            acc: Acceleration in m/s²
            dec: Deceleration in m/s²
            target_force:  Target force
            force_limit: Force limit
        """
        # Format:  choice,targetPos,maxVel,acc,dec,targetForce,forceLimit\n
        cmd = f"{choice},{target_pos},{max_vel},{acc},{dec},{target_force},{force_limit}\n"
        print(f"\n→ Sending:  {cmd. strip()}")
        self.ser.write(cmd.encode('utf-8'))
        time.sleep(0.1)  # Give Arduino time to process
        
        # Read Arduino's echo/debug output
        self.read_responses(duration=0.5)
    
    def read_responses(self, duration=1.0):
        """Read and print responses from Arduino for specified duration"""
        start_time = time.time()
        responses = []
        
        while time.time() - start_time < duration:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"← Arduino: {line}")
                    responses.append(line)
            time.sleep(0.01)
        
        return responses
    
    def get_position(self):
        """Request current position from actuator"""
        print("\n=== Getting Position ===")
        self.send_command(LinmotCommand.GET_POSITION)
        
        # Wait for position response
        start_time = time.time()
        while time.time() - start_time < 2.0:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"← {line}")
                
                # Parse position response:  "Sending:  pos,statusWord"
                if line.startswith("Sending:"):
                    try:
                        data = line.replace("Sending:", "").strip()
                        parts = data.split(',')
                        if len(parts) >= 2:
                            position = float(parts[0])
                            status = parts[1]
                            print(f"✓ Position: {position:. 4f} mm, Status: {status}")
                            return position, status
                    except (ValueError, IndexError) as e:
                        print(f"✗ Error parsing response: {e}")
            time.sleep(0.01)
        
        print("✗ No position response received")
        return None, None
    
    def test_initialization(self):
        """Test 1: Initialize and prepare the drive"""
        print("\n" + "="*50)
        print("TEST 1: Initialization Sequence")
        print("="*50)
        
        print("\n--- Setting Operational Mode ---")
        self.send_command(LinmotCommand.OPERATIONAL_MODE)
        time.sleep(1)
        
        print("\n--- Switching On Drive ---")
        self.send_command(LinmotCommand. SWITCH_ON)
        time.sleep(1)
        
        print("\n✓ Initialization complete")
    
    def test_homing(self):
        """Test 2: Home the actuator"""
        print("\n" + "="*50)
        print("TEST 2: Homing")
        print("="*50)
        
        self.send_command(LinmotCommand.HOMING)
        print("\nWaiting for homing to complete (10 seconds)...")
        
        for i in range(10):
            time.sleep(1)
            print(f"  {10-i} seconds remaining...")
            self.read_responses(duration=0.1)
        
        print("✓ Homing sequence sent")
    
    def test_position_control(self):
        """Test 3: Position control with various parameters"""
        print("\n" + "="*50)
        print("TEST 3: Position Control")
        print("="*50)
        
        test_moves = [
            {"pos": 10.0, "vel": 0.1, "acc": 1.0, "dec": 1.0, "desc": "Move to 10mm"},
            {"pos": 20.0, "vel": 0.2, "acc": 2.0, "dec": 2.0, "desc": "Move to 20mm"},
            {"pos": 0.0, "vel": 0.15, "acc": 1.5, "dec": 1.5, "desc": "Return to 0mm"},
        ]
        
        for move in test_moves:
            print(f"\n--- {move['desc']} ---")
            self.send_command(
                LinmotCommand.GOTO_POSITION,
                target_pos=move['pos'],
                max_vel=move['vel'],
                acc=move['acc'],
                dec=move['dec']
            )
            
            # Wait for movement
            time.sleep(3)
            
            # Check position
            pos, status = self.get_position()
            if pos is not None:
                error = abs(pos - move['pos'])
                if error < 1.0:  # Within 1mm tolerance
                    print(f"✓ Position reached (error: {error:.4f} mm)")
                else:
                    print(f"⚠ Position error:  {error:.4f} mm")
    
    def test_force_control(self):
        """Test 4: Force control features"""
        print("\n" + "="*50)
        print("TEST 4: Force Control")
        print("="*50)
        
        print("\n--- Move with Force Control ---")
        self.send_command(
            LinmotCommand.GOTO_POS_WITH_FORCE,
            target_pos=15.0,
            max_vel=0.1,
            acc=1.0,
            dec=1.0,
            target_force=50.0,
            force_limit=100.0
        )
        time.sleep(3)
        self.get_position()
        
        print("\n--- Change Target Force ---")
        self.send_command(
            LinmotCommand.CHANGE_TARGET_FORCE,
            target_force=30.0
        )
        time.sleep(2)
    
    def test_continuous_monitoring(self, duration=10):
        """Test 5: Continuous position monitoring"""
        print("\n" + "="*50)
        print(f"TEST 5: Continuous Monitoring ({duration} seconds)")
        print("="*50)
        
        start_time = time.time()
        positions = []
        
        while time.time() - start_time < duration:
            pos, status = self.get_position()
            if pos is not None:
                positions.append((time.time() - start_time, pos))
            time.sleep(0.5)
        
        print(f"\n✓ Collected {len(positions)} position samples")
        if positions:
            print("\nPosition history:")
            for t, pos in positions:
                print(f"  t={t:.1f}s: {pos:.4f} mm")
    
    def test_shutdown(self):
        """Test 6: Safe shutdown"""
        print("\n" + "="*50)
        print("TEST 6: Shutdown Sequence")
        print("="*50)
        
        print("\n--- Returning to Home ---")
        self.send_command(
            LinmotCommand.GOTO_POSITION,
            target_pos=0.0,
            max_vel=0.1,
            acc=1.0,
            dec=1.0
        )
        time.sleep(3)
        
        print("\n--- Switching Off Drive ---")
        self.send_command(LinmotCommand.SWITCH_OFF)
        time.sleep(1)
        
        print("✓ Shutdown complete")
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("\n" + "="*60)
        print("LINMOT CONTROLLER - FULL TEST SUITE")
        print("="*60)
        
        try:
            self.test_initialization()
            time.sleep(2)
            
            self.test_homing()
            time.sleep(2)
            
            self.test_position_control()
            time.sleep(2)
            
            # self.test_force_control()
            # time.sleep(2)
            
            self.test_continuous_monitoring(duration=5)
            time.sleep(2)
            
            self.test_shutdown()
            
            print("\n" + "="*60)
            print("✓ ALL TESTS COMPLETED SUCCESSFULLY")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n\n⚠ Tests interrupted by user")
            self.test_shutdown()
        except Exception as e:
            print(f"\n\n✗ Test failed with error: {e}")
            import traceback
            traceback.print_exc()
    
    def close(self):
        """Close serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("\n✓ Serial connection closed")


def main():
    parser = argparse.ArgumentParser(description='Test Arduino Linmot Controller')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--test', choices=['all', 'init', 'homing', 'position', 'force', 'monitor'], 
                        default='all', help='Specific test to run')
    
    args = parser.parse_args()
    
    tester = LinmotTester(port=args.port, baudrate=args.baudrate)
    
    try:
        if args.test == 'all':
            tester.run_all_tests()
        elif args.test == 'init':
            tester.test_initialization()
        elif args.test == 'homing': 
            tester.test_homing()
        elif args.test == 'position':
            tester.test_position_control()
        elif args.test == 'force':
            tester.test_force_control()
        elif args.test == 'monitor': 
            tester.test_continuous_monitoring(duration=30)
    
    finally:
        tester.close()


if __name__ == '__main__':
    main()