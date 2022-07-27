#!/usr/bin/env python3
# Importing Libraries
import serial
import getch
import serial.tools.list_ports
import sys, termios, tty, os, time

def getBlackPillPort():
    return "/dev/ttyTHS2"
    portList = list(serial.tools.list_ports.comports())
    for port in portList:
        if "BLACKPILL" in port[1]:
            return port[0]

if __name__ == "__main__":

    print("These are the keys used to control the arm:\n\
            W, S for the upper arm,\n\
            A, D for the base hinge,\n\
            T, G for the forearm,\n\
            F, H for the wrist,\n\
            I, K for the claw base,\n\
            J, L for the claws,\n\
            Esc to end program\n\
    ")

    blackpill = arduino = serial.Serial(port=getBlackPillPort(), baudrate=115200)

    while True:
        char = getch.getch()
        if ord(char)==27:
            print("Exiting")
            exit(0)
        
        blackpill.write(char.encode())
        time.sleep(0.05)
