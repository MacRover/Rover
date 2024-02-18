#!/usr/bin/env python3

## This program reads the serial input coming from the esp32 used to detect beacon

#!/usr/bin/env python3
# Importing Libraries
import serial
import serial.tools.list_ports
import time
import struct
import sys

port = "/dev/ttyUSB0"


if __name__ == "__main__":
    
    if(len(sys.argv) > 1):
        port = sys.argv[1]


    blackpill = arduino = serial.Serial(port=port, baudrate=115200)
    print("Connected to ESP-32 at port", port)

    while True:
        data = arduino.readline().decode('utf-8')
        print(data)
