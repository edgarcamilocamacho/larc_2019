#!/usr/bin/env python

import sys
import numpy as np
import yaml

import rospy
import rospkg
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from dynamixel_controllers.srv import TorqueEnable

from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow, QFileDialog, QWidget
from PyQt5.QtWidgets import QMessageBox, QTableWidget, QTableWidgetItem, QPushButton

from PyQt5.uic import loadUi
from PyQt5 import QtCore
from PyQt5.QtCore import QTimer, QRegExp
from PyQt5.QtGui import QPixmap, QImage, QRegExpValidator

class LarcGuiController(QMainWindow):
    def __init__(self):
        super(LarcGuiController, self).__init__(None)
        rospack = rospkg.RosPack()
        self.larc_settings_path = rospack.get_path('larc_control')
        loadUi(self.larc_settings_path+'/config/larc_controller.ui', self)

        ### LOCAL VARIABLES
        self.key_status = { QtCore.Qt.Key_A:False, 
                            QtCore.Qt.Key_W:False, 
                            QtCore.Qt.Key_D:False, 
                            QtCore.Qt.Key_S:False,
                            QtCore.Qt.Key_Q:False,
                            QtCore.Qt.Key_E:False, }
        self.joint_states = None
        
        ### ROS ###
        rospy.init_node('larc_controller')
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Vector3, queue_size = 1)
        self.pub_base = rospy.Publisher("/rotating_base/command", Float64, queue_size = 1)
        self.pub_zipper = rospy.Publisher("/zipper/command", Float64, queue_size = 1)
        self.pub_shoulder = rospy.Publisher("/shoulder/command", Float64, queue_size = 1)
        self.pub_elbow = rospy.Publisher("/elbow/command", Float64, queue_size = 1)
        self.pub_wrist_x = rospy.Publisher("/wrist_x/command", Float64, queue_size = 1)
        self.pub_wrist_y = rospy.Publisher("/wrist_y/command", Float64, queue_size = 1)
        self.pub_gripper = rospy.Publisher("/gripper/command", Float64, queue_size = 1)

        self.srv_tor_base = rospy.ServiceProxy('/rotating_base/torque_enable', TorqueEnable)
        self.srv_tor_zipper = rospy.ServiceProxy('/zipper/torque_enable', TorqueEnable)
        self.srv_tor_shoulder = rospy.ServiceProxy('/shoulder/torque_enable', TorqueEnable)
        self.srv_tor_elbow = rospy.ServiceProxy('/elbow/torque_enable', TorqueEnable)
        self.srv_tor_wrist_x = rospy.ServiceProxy('/wrist_x/torque_enable', TorqueEnable)
        self.srv_tor_wrist_y = rospy.ServiceProxy('/wrist_y/torque_enable', TorqueEnable)
        self.srv_tor_gripper = rospy.ServiceProxy('/gripper/torque_enable', TorqueEnable)

        self.sub_state_base = rospy.Subscriber('/joint_states', JointState, self.sub_state_base_callback)
        
        ### OBJECTS
        ## Timers
        self.timer_keys = QTimer()
        self.timer_keys.timeout.connect(self.timer_keys_timeout)
        self.timer_keys.start(100)
        ## Spins
        # self.spin_linear_speed.valueChanged.connect(self.spin_linear_speed_valueChanged)
        ## Buttons
        self.btn_control.clicked.connect(self.btn_control_clicked)
        self.btn_update.clicked.connect(self.btn_update_clicked)
        self.btn_pub_base.clicked.connect(self.btn_pub_base_clicked)
        self.btn_pub_zipper.clicked.connect(self.btn_pub_zipper_clicked)
        self.btn_pub_shoulder.clicked.connect(self.btn_pub_shoulder_clicked)
        self.btn_pub_elbow.clicked.connect(self.btn_pub_elbow_clicked)
        self.btn_pub_wrist_x.clicked.connect(self.btn_pub_wrist_x_clicked)
        self.btn_pub_wrist_y.clicked.connect(self.btn_pub_wrist_y_clicked)
        self.btn_pub_gripper.clicked.connect(self.btn_pub_gripper_clicked)
        ## Checks
        self.check_tor_base.stateChanged.connect(self.check_tor_base_stateChanged)
        self.check_tor_zipper.stateChanged.connect(self.check_tor_zipper_stateChanged)
        self.check_tor_shoulder.stateChanged.connect(self.check_tor_shoulder_stateChanged)
        self.check_tor_elbow.stateChanged.connect(self.check_tor_elbow_stateChanged)
        self.check_tor_wrist_x.stateChanged.connect(self.check_tor_wrist_x_stateChanged)
        self.check_tor_wrist_y.stateChanged.connect(self.check_tor_wrist_y_stateChanged)
        self.check_tor_gripper.stateChanged.connect(self.check_tor_gripper_stateChanged)

        self.setFocus()

    ### ROS Callbacks
    def sub_state_base_callback(self, msg):
        self.joint_states = msg
    
    ### SLOTS
    ## Buttons
    def btn_pub_base_clicked(self):
        self.pub_base.publish(self.spin_base.value())
    def btn_pub_zipper_clicked(self):
        self.pub_zipper.publish(self.spin_zipper.value())
    def btn_pub_shoulder_clicked(self):
        self.pub_shoulder.publish(self.spin_shoulder.value())
    def btn_pub_elbow_clicked(self):
        self.pub_elbow.publish(self.spin_elbow.value())
    def btn_pub_wrist_x_clicked(self):
        self.pub_wrist_x.publish(self.spin_wrist_x.value())
    def btn_pub_wrist_y_clicked(self):
        self.pub_wrist_y.publish(self.spin_wrist_y.value())   
    def btn_pub_gripper_clicked(self):
        self.pub_gripper.publish(self.spin_gripper.value())   
    def btn_control_clicked(self):
        self.setFocus()
    def btn_update_clicked(self):
        for i, name in enumerate(self.joint_states.name):
            if name=='base':
                self.spin_base.setValue(round(self.joint_states.position[i],2))
            if name=='zipper':
                self.spin_zipper.setValue(round(self.joint_states.position[i],3))
            if name=='shoulder':
                self.spin_shoulder.setValue(round(self.joint_states.position[i],2))
            if name=='elbow':
                self.spin_elbow.setValue(round(self.joint_states.position[i],2))
            if name=='wrist_x':
                self.spin_wrist_x.setValue(round(self.joint_states.position[i],2))
            if name=='wrist_y':
                self.spin_wrist_y.setValue(round(self.joint_states.position[i],2))
            if name=='gripper':
                self.spin_gripper.setValue(round(self.joint_states.position[i],2))
    ## Checks
    def check_tor_base_stateChanged(self, state):
        self.srv_tor_base(state)
    def check_tor_zipper_stateChanged(self, state):
        self.srv_tor_zipper(state)
    def check_tor_shoulder_stateChanged(self, state):
        self.srv_tor_shoulder(state)
    def check_tor_elbow_stateChanged(self, state):
        self.srv_tor_elbow(state)
    def check_tor_wrist_x_stateChanged(self, state):
        self.srv_tor_wrist_x(state)
    def check_tor_wrist_y_stateChanged(self, state):
        self.srv_tor_wrist_y(state)
    def check_tor_gripper_stateChanged(self, state):
        self.srv_tor_gripper(state)
    ## Spine slots
    
    ## Timer slots
    def timer_keys_timeout(self):
        linear_speed = self.spin_linear_speed.value()
        angular_speed = self.spin_angular_speed.value()
        if self.key_status[QtCore.Qt.Key_W]:
            self.pub_cmd_vel.publish(0.0, linear_speed, 0.0)
        elif self.key_status[QtCore.Qt.Key_S]:
            self.pub_cmd_vel.publish(0.0, -linear_speed, 0.0)
        elif self.key_status[QtCore.Qt.Key_A]:
            self.pub_cmd_vel.publish(-linear_speed, 0.0, 0.0)
        elif self.key_status[QtCore.Qt.Key_D]:
            self.pub_cmd_vel.publish(linear_speed, 0.0, 0.0)
        elif self.key_status[QtCore.Qt.Key_Q]:
            self.pub_cmd_vel.publish(0.0, 0.0, -angular_speed)
        elif self.key_status[QtCore.Qt.Key_E]:
            self.pub_cmd_vel.publish(0.0, 0.0, angular_speed)
        else:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
    
    ### EVENTS
    def keyPressEvent(self, event):
        key = event.key()
        if key in self.key_status:
            self.key_status[key] = True
    
    def keyReleaseEvent(self, event):
        key = event.key()
        if key in self.key_status:
            self.key_status[key] = False

app = QApplication(sys.argv)
ui = LarcGuiController()
ui.setWindowTitle('LARC Controller')
ui.show()   
sys.exit(app.exec_())