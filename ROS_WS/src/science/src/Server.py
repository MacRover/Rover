#! /usr/bin/env python
import rospy
import science.srv
from std_msgs.msg import UInt8

def move_drill(req):
  try:
    if(req.command == 0):
        print("moving drill anti-clockwise")
    elif(req.command == 1):
        print("moving drill clockwise")
    else:
        print("stopping drill")
    return science.srv.DrillResponse(success=True)
  except:
    return science.srv.DrillResponse(success=False)
  
def move_carousel(req):
  try:
    if(req.command == 0):
        print("moving carousel anti-clockwise")
    elif(req.command == 1):
        print("moving carousel clockwise")
    return science.srv.CarouselResponse(success=True)
  except:
    return science.srv.CarouselResponse(success=False)

def move_auger(msg):
  if(msg.data == 0):
     print("moving auger up")
  elif(msg.data == 1):
    print("moving auger down")
   

rospy.init_node("science_server")
print("Started science server")

drill_service = rospy.Service("science/drill", science.srv.Drill, move_drill)
carousel_service = rospy.Service("science/carousel", science.srv.Carousel, move_carousel)
rospy.Subscriber('science/auger', UInt8, move_auger)

rospy.spin()