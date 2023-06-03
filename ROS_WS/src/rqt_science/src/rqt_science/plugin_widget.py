#!/usr/bin/env python

import os
import rospkg

import rospy
import science.srv
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
        # self.remove_site = rospy.ServiceProxy("/science/sci/delete_site", rover_science.srv.DeleteSite)
        # self.take_measurement = rospy.ServiceProxy("/science/sci/take_measurement", rover_science.srv.TakeMeasurement)
        # self.retake_measurement = rospy.ServiceProxy("/science/sci/update_measurement", rover_science.srv.UpdateMeasurement)
        # self.change_site_name = rospy.ServiceProxy("/science/sci/change_site_name", rover_science.srv.SiteNameChange)
        self.drillLeftPushButton.clicked.connect(self.move_drill_left)
        # self.removeSitePushButton.clicked.connect(self.remove_site_click)
        # self.listWidget.currentRowChanged.connect(self.selected_change)
        # self.takeMeasurementPushButton.clicked.connect(self.new_measurement_clicked)
        # self.renamePushButton.clicked.connect(self.rename_clicked)
        # self.last_site_data = None

        context.add_widget(self)

    def move_drill_left(self):
        self.move_drill(1)


    def shutdown(self):
        pass