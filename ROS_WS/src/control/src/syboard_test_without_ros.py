#!/usr/bin/env python3
# Importing Libraries
import serial
import serial.tools.list_ports
import time
import struct

def getBlackPillPort():
    return "/dev/ttyTHS2"
    portList = list(serial.tools.list_ports.comports())
    for port in portList:
        if "BLACKPILL" in port[1]:
            return port[0]

if __name__ == "__main__":

    blackpill = arduino = serial.Serial(port=getBlackPillPort(), baudrate=115200)

    while True:
        command = input("Enter a command number: ")
        if command == "exit":
            print("Exiting")
            exit(0)

        command_int = int(command)
        data = struct.pack("<B", command_int)
        arduino.write(data)
        time.sleep(0.05)
        data = arduino.read()
        print(data)
