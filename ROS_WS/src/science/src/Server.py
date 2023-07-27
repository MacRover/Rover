#! /usr/bin/env python
import rospy
import serial
import science.srv
from std_msgs.msg import UInt8

micro = serial.Serial(port="/dev/ttyACM1", baudrate=115200)

def move_drill(req):
  try:
    if(req.command == 0):
        micro.write('f'.encode())
        print("moving drill anti-clockwise")
    elif(req.command == 1):
        print("moving drill clockwise")
        micro.write('g'.encode())
    else:
        print("stopping drill")
        micro.write('h'.encode())
    return science.srv.DrillResponse(success=True)
  except:
    return science.srv.DrillResponse(success=False)
  
def move_carousel(req):
  try:
    if(req.command == 0):
        print("moving carousel anti-clockwise")
        micro.write('q'.encode())
    elif(req.command == 1):
        print("moving carousel clockwise")
        micro.write('e'.encode())
    else:
        print("stopping auger")
        micro.write('s'.encode())
    return science.srv.CarouselResponse(success=True)
  except:
    return science.srv.CarouselResponse(success=False)

def move_auger(msg):
  if(msg.data == 0):
    print("moving auger up")
    micro.write('a'.encode())
  elif(msg.data == 1):
    print("moving auger down")
    micro.write('d'.encode())
   
rospy.init_node("science_server")
print("Started science server")

drill_service = rospy.Service("science/drill", science.srv.Drill, move_drill)
carousel_service = rospy.Service("science/carousel", science.srv.Carousel, move_carousel)
rospy.Subscriber('science/auger', UInt8, move_auger)

rospy.spin()