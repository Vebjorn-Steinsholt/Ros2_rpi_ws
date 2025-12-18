#!/usr/bin/env python3
import serial
import time

time_last_cmd_sent = time.time()
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

try:
    while True:
        time.sleep(0.01)
        if ser.in_waiting > 0:
            msg = ser.readline().decode('utf-8').rstrip()
            print(msg)
        time_now = time.time()
        if time_now-time_last_cmd_sent > 1.0:
            time_last_cmd_sent = time_now
            ser.write("test\n".encode('utf-8'))
except KeyboardInterrupt:
    print("Closing serial port...")
    ser.close()
    print("Exit program.")