#!/usr/bin/env python

import os
import rospkg

import rospy
import science.srv
from std_msgs.msg import UInt8
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtWidgets import QWidget, QInputDialog
from python_qt_binding.QtGui import QPixmap


class ScienceWidget(QWidget):

    def __init__(self, context):
        super(ScienceWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_science'), 'resource', 'science.ui')
        loadUi(ui_file, self)

        self.move_drill = rospy.ServiceProxy("/science/drill", science.srv.Drill)
        self.move_carousel = rospy.ServiceProxy("/science/carousel", science.srv.Carousel)
        self.auger_publisher = rospy.Publisher("/science/auger", UInt8, queue_size=10)

        self.drillLeftPushButton.clicked.connect(self.move_drill_left)
        self.drillRightPushButton.clicked.connect(self.move_drill_right)
        self.drillStopPushButton.clicked.connect(self.move_drill_stop)
        self.augerDownPushButton.clicked.connect(self.move_auger_up)
        self.augerUpPushButton.clicked.connect(self.move_auger_down)
        self.augerStopPushButton.clicked.connect(self.move_auger_stop)
        self.carouselRightPushButton.clicked.connect(self.move_carousel_right)
        self.carouselLeftPushButton.clicked.connect(self.move_carousel_left)

        context.add_widget(self)

    def move_drill_left(self):
        self.move_drill(0)

    def move_drill_right(self):
        self.move_drill(1)

    def move_drill_stop(self):
        self.move_drill(2)

    def move_auger_up(self):
        self.auger_publisher.publish(0)

    def move_auger_down(self):
        self.auger_publisher.publish(1)

    def move_auger_stop(self):
        self.move_carousel(2)

    def move_carousel_left(self):
        self.move_carousel(0)

    def move_carousel_right(self):
        self.move_carousel(1)


    def shutdown(self):
        pass