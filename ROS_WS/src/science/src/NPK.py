#!/usr/bin/env python
import rospy
import serial
import serial.tools.list_ports
import minimalmodbus
import time
from science.msg import NPK


def get_port():
    ports = []
    result = []
    
    comports = serial.tools.list_ports.comports()
    for p in comports:
        ports.append(str(p).split(" ")[0])

    if ports == []:
        print("Could not find any ports!")
        exit()

    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except(serial.SerialException):
            pass
    
    if len(result) == 0:
        print("No usable ports found")
        exit()
        
    elif len(result) == 1:
        return result[0]

    else:
        print("Found the following usable ports. \nPlease type the index to select an option:")
    
        for i in range(len(result)):
            print("{}. {}".format(i, result[i]))

        choice = input()
        if type(choice) != int:
            print("Invalid index chosen!")
            exit()
        elif choice < 0 or choice > len(result)-1:
            print("Invalid index chosen!")
            exit()

        try:
            return result[choice]
        except:
            print("Could not get index!")
            exit()
            

port = get_port()

rospy.init_node("npk_publisher", anonymous=True)
npk_publisher = rospy.Publisher('/npk', NPK, queue_size=10)

instrument = minimalmodbus.Instrument(port, 1, debug=False)
instrument.serial.baudrate = 9600
instrument.serial.bytesize = 8
instrument.serial.parity = serial.PARITY_NONE
instrument.serial.stopbits = 1

instrument.mode = minimalmodbus.MODE_RTU

print(instrument)

while True:

    ph = instrument.read_register(6, 1)
    temp = instrument.read_register(19, 1)
    conductivity = instrument.read_register(21, 1)
    nitrogen = instrument.read_register(30, 1)
    phosphorus = instrument.read_register(31, 1)
    potassium = instrument.read_register(32, 1)

    print("--------------NEW SAMPLE-----------------")
    print("ph: {}".format(ph))
    print("temp: {}".format(temp))
    print("conductivity: {}".format(conductivity))
    print("nitrogen: {}".format(nitrogen))
    print("phosphorus: {}".format(phosphorus))
    print("potassium: {}".format(potassium))
    #print("baud rate: {}".format(baudrate))
    time.sleep(1)