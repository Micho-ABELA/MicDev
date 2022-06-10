import os
import rospy
import rospkg
from airplane_master.msg import custom
from std_msgs.msg import String

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from PyQt5 import QtWidgets


class MyWidget(QWidget):
    def __init__(self, context):
        super(MyWidget, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('airplane_master'), 'resource', 'GUI.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self)
        self.sensor_counter = 1  # for sensor list names
        self.InitUI()
        rospy.Subscriber('data_topic', custom, self.callback)  # subscribe to the data Topic from PiMaster
        self.pub = rospy.Publisher('topic_quit', String, queue_size=10)  # create a Publisher to share Quit event

    def InitUI(self):
        self.Connection_Label.setText("Connecting...")
        self.Connection_Label.setStyleSheet("background-color: yellow")
        self.Exit_Button.setStyleSheet("background-color: red")
        self.Exit_Button.setText("EXIT")
        self.Exit_Button.clicked[bool].connect(self.Exit_Event)

    def Exit_Event(self):
        self.pub.publish("1")  # send EXIT message
        self.close()

    def callback(self, data):  # This is the CallBack function called after subscribing the topic, to use the data
        column_counter = 0
        if self.sensor_counter == 1:  # write sensor list the first time only
            self.Sensor_List.addItem(QtWidgets.QListWidgetItem(" \n \n // Sensor Names //\n"))
            for sensor in data.sensor_names:
                self.Sensor_List.addItem(QtWidgets.QListWidgetItem("S{}: {}".format(self.sensor_counter, sensor)))
                self.sensor_counter += 1

        for temperature in data.temperature_list:  # fill in the Temperature
            self.Sensor_Table.setItem(0, column_counter, QtWidgets.QTableWidgetItem("  {}".format(temperature)))
            column_counter += 1
        column_counter = 0
        for pressure in data.pressure_list:  # fill in the Pressure
            self.Sensor_Table.setItem(1, column_counter, QtWidgets.QTableWidgetItem("  {}".format(pressure)))
            column_counter += 1
        column_counter = 0
        for altitude in data.altitude_list:  # # fill in the Altitude
            self.Sensor_Table.setItem(2, column_counter, QtWidgets.QTableWidgetItem("  {}".format(altitude)))
            column_counter += 1
        if data.connection_status:
            self.Connection_Label.setText("Connected")
            self.Connection_Label.setStyleSheet("background-color: green")
        else:
            self.Connection_Label.setText("Disconnected")
            self.Connection_Label.setStyleSheet("background-color: red")
