#!/usr/bin/env python

import sys
import numpy as np
import yaml
import time

import rospy
import tf
import rospkg
from std_msgs.msg import Float64, String, Float32MultiArray
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
        self.L = 0

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
        self.pub_move = rospy.Publisher("/larc_movement/go_to_pos", Float32MultiArray, queue_size = 1)

        self.srv_tor_base = rospy.ServiceProxy('/rotating_base/torque_enable', TorqueEnable)
        self.srv_tor_zipper = rospy.ServiceProxy('/zipper/torque_enable', TorqueEnable)
        self.srv_tor_shoulder = rospy.ServiceProxy('/shoulder/torque_enable', TorqueEnable)
        self.srv_tor_elbow = rospy.ServiceProxy('/elbow/torque_enable', TorqueEnable)
        self.srv_tor_wrist_x = rospy.ServiceProxy('/wrist_x/torque_enable', TorqueEnable)
        self.srv_tor_wrist_y = rospy.ServiceProxy('/wrist_y/torque_enable', TorqueEnable)
        self.srv_tor_gripper = rospy.ServiceProxy('/gripper/torque_enable', TorqueEnable)

        self.sub_state_base = rospy.Subscriber('/joint_states', JointState, self.sub_state_base_callback)
        
        self.tf_listener = tf.TransformListener()
        self.ros_rate = rospy.Rate(10.0)

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
        self.btn_update_l.clicked.connect(self.btn_update_l_clicked)
        self.btn_pub_base.clicked.connect(self.btn_pub_base_clicked)
        self.btn_pub_zipper.clicked.connect(self.btn_pub_zipper_clicked)
        self.btn_pub_shoulder.clicked.connect(self.btn_pub_shoulder_clicked)
        self.btn_pub_elbow.clicked.connect(self.btn_pub_elbow_clicked)
        self.btn_pub_wrist_x.clicked.connect(self.btn_pub_wrist_x_clicked)
        self.btn_pub_wrist_y.clicked.connect(self.btn_pub_wrist_y_clicked)
        self.btn_pub_gripper.clicked.connect(self.btn_pub_gripper_clicked)
        self.btn_copy_1.clicked.connect(self.btn_copy_1_clicked)
        self.btn_copy_2.clicked.connect(self.btn_copy_2_clicked)
        self.btn_copy_3.clicked.connect(self.btn_copy_3_clicked)
        self.btn_copy_4.clicked.connect(self.btn_copy_4_clicked)
        self.btn_copy_5.clicked.connect(self.btn_copy_5_clicked)
        self.btn_send_1.clicked.connect(self.btn_send_1_clicked)
        self.btn_send_2.clicked.connect(self.btn_send_2_clicked)
        self.btn_send_3.clicked.connect(self.btn_send_3_clicked)
        self.btn_send_4.clicked.connect(self.btn_send_4_clicked)
        self.btn_send_5.clicked.connect(self.btn_send_5_clicked)
        self.btn_se_pp.clicked.connect(self.btn_pr_clicked)
        self.btn_se_p.clicked.connect(self.btn_pr_clicked)
        self.btn_sw_pp.clicked.connect(self.btn_pr_clicked)
        self.btn_sw_p.clicked.connect(self.btn_pr_clicked)
        self.btn_se_rr.clicked.connect(self.btn_pr_clicked)
        self.btn_se_r.clicked.connect(self.btn_pr_clicked)
        self.btn_sw_rr.clicked.connect(self.btn_pr_clicked)
        self.btn_sw_r.clicked.connect(self.btn_pr_clicked)
        self.btn_open.clicked.connect(self.btn_open_clicked)
        self.btn_close.clicked.connect(self.btn_close_clicked)
        self.btn_clear.clicked.connect(self.btn_clear_clicked)
        ## Checks
        self.check_tor_base.stateChanged.connect(self.check_tor_base_stateChanged)
        self.check_tor_zipper.stateChanged.connect(self.check_tor_zipper_stateChanged)
        self.check_tor_shoulder.stateChanged.connect(self.check_tor_shoulder_stateChanged)
        self.check_tor_elbow.stateChanged.connect(self.check_tor_elbow_stateChanged)
        self.check_tor_wrist_x.stateChanged.connect(self.check_tor_wrist_x_stateChanged)
        self.check_tor_wrist_y.stateChanged.connect(self.check_tor_wrist_y_stateChanged)
        self.check_tor_gripper.stateChanged.connect(self.check_tor_gripper_stateChanged)
        self.check_all_tor.stateChanged.connect(self.check_all_tor_stateChanged)
        ## Sliders
        self.slider_dx.valueChanged.connect(self.slider_dx_valueChanged)

        self.setFocus()
    
    def pos_to_list(self):
        str_list = "["
        str_list += str(self.spin_base.value()) + ", "
        str_list += str(self.spin_zipper.value()) + ", "
        str_list += str(self.spin_shoulder.value()) + ", "
        str_list += str(self.spin_elbow.value()) + ", "
        str_list += str(self.spin_wrist_x.value()) + ", "
        str_list += str(self.spin_wrist_y.value()) + "]"
        return str_list
    ### ROS Callbacks
    def sub_state_base_callback(self, msg):
        self.joint_states = msg
    
    ### SLOTS
    ## Sliders
    def slider_dx_valueChanged(self, value):
        dx = value/1000.0
        alpha = np.arctan(dx/self.L)
        dl = np.sqrt(dx**2+self.L**2)-self.L
        self.pub_base.publish(-alpha)
        self.pub_wrist_y.publish(-1.57-alpha)  
        self.pub_zipper.publish(dl)
    ## Buttons
    def btn_clear_clicked(self):
        self.txt_list_1.clear()
        self.txt_list_2.clear()
        self.txt_list_3.clear()
        self.txt_list_4.clear()
        self.txt_list_5.clear()

    def btn_open_clicked(self):
        self.pub_gripper.publish(0.35)
        time.sleep(0.2)
        self.pub_gripper.publish(0.40)
        time.sleep(0.2)
        self.pub_gripper.publish(0.65)

    def btn_close_clicked(self):
        self.pub_gripper.publish(0.27)


    def btn_pr_clicked(self):
        btn_names = self.sender().text()
        if btn_names == 'SE++':
            self.spin_shoulder.setValue(self.spin_shoulder.value()+0.1)
            self.spin_elbow.setValue(self.spin_elbow.value()+0.1)
        elif btn_names == 'SE--':
            self.spin_shoulder.setValue(self.spin_shoulder.value()-0.1)
            self.spin_elbow.setValue(self.spin_elbow.value()-0.1)
        elif btn_names == 'SE+':
            self.spin_shoulder.setValue(self.spin_shoulder.value()+0.01)
            self.spin_elbow.setValue(self.spin_elbow.value()+0.01)
        elif btn_names == 'SE-':
            self.spin_shoulder.setValue(self.spin_shoulder.value()-0.01)
            self.spin_elbow.setValue(self.spin_elbow.value()-0.01)
        elif btn_names == 'SW++':
            self.spin_shoulder.setValue(self.spin_shoulder.value()+0.1)
            self.spin_wrist_x.setValue(self.spin_wrist_x.value()+0.1)
        elif btn_names == 'SW--':
            self.spin_shoulder.setValue(self.spin_shoulder.value()-0.1)
            self.spin_wrist_x.setValue(self.spin_wrist_x.value()-0.1)
        elif btn_names == 'SW+':
            self.spin_shoulder.setValue(self.spin_shoulder.value()+0.01)
            self.spin_wrist_x.setValue(self.spin_wrist_x.value()+0.01)
        elif btn_names == 'SW-':
            self.spin_shoulder.setValue(self.spin_shoulder.value()-0.01)
            self.spin_wrist_x.setValue(self.spin_wrist_x.value()-0.01)
            
    def btn_send_1_clicked(self):
        jv = eval(self.txt_list_1.toPlainText())
        msg = Float32MultiArray()
        msg.data = jv
        self.pub_move.publish(msg)
    def btn_send_2_clicked(self):
        jv = eval(self.txt_list_2.toPlainText())
        msg = Float32MultiArray()
        msg.data = jv
        self.pub_move.publish(msg)
    def btn_send_3_clicked(self):
        jv = eval(self.txt_list_3.toPlainText())
        msg = Float32MultiArray()
        msg.data = jv
        self.pub_move.publish(msg)
    def btn_send_4_clicked(self):
        jv = eval(self.txt_list_4.toPlainText())
        msg = Float32MultiArray()
        msg.data = jv
        self.pub_move.publish(msg)
    def btn_send_5_clicked(self):
        jv = eval(self.txt_list_5.toPlainText())
        msg = Float32MultiArray()
        msg.data = jv
        self.pub_move.publish(msg)

    def btn_copy_1_clicked(self):
        self.txt_list_1.clear()
        self.txt_list_1.insertPlainText(self.pos_to_list())
    def btn_copy_2_clicked(self):
        self.txt_list_2.clear()
        self.txt_list_2.insertPlainText(self.pos_to_list())
    def btn_copy_3_clicked(self):
        self.txt_list_3.clear()
        self.txt_list_3.insertPlainText(self.pos_to_list())
    def btn_copy_4_clicked(self):
        self.txt_list_4.clear()
        self.txt_list_4.insertPlainText(self.pos_to_list())
    def btn_copy_5_clicked(self):
        self.txt_list_5.clear()
        self.txt_list_5.insertPlainText(self.pos_to_list())
    def btn_update_l_clicked(self):
        while True:
            try:
                (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/link_wrist_x', rospy.Time(0))
                print( "Distance in y is = {0:f}".format( abs(trans[1]) ) )
                self.L = abs(trans[1])
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.ros_rate.sleep()
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
            if name=='rotating_base':
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
    def check_all_tor_stateChanged(self, state):
        self.check_tor_base.setCheckState(state)
        self.check_tor_zipper.setCheckState(state)
        self.check_tor_shoulder.setCheckState(state)
        self.check_tor_elbow.setCheckState(state)
        self.check_tor_wrist_x.setCheckState(state)
        self.check_tor_wrist_y.setCheckState(state)
        self.check_tor_gripper.setCheckState(state)
        
    ## Spine slots
    
    ## Timer slots
    def timer_keys_timeout(self):
        if self.check_pub_cmd_vel.checkState():
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