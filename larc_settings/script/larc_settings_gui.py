#!/usr/bin/env python

import sys
import numpy as np
import yaml

import rospy
import rospkg
from std_msgs.msg import Float64, String
from larc_settings.msg import LarcSettings

from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow, QFileDialog, QWidget
from PyQt5.QtWidgets import QMessageBox, QTableWidget, QTableWidgetItem, QPushButton

from PyQt5.uic import loadUi
from PyQt5.QtCore import QTimer, QRegExp
from PyQt5.QtGui import QPixmap, QImage, QRegExpValidator

class LarcSettingsGui(QMainWindow):
    def __init__(self):
        super(LarcSettingsGui, self).__init__(None)

        rospack = rospkg.RosPack()
        self.larc_settings_path = rospack.get_path('larc_settings')
        self.config_path = self.larc_settings_path + '/config/config.yaml'

        loadUi(self.larc_settings_path+'/config/larc_settings_gui.ui', self)

        with open(self.config_path, 'r') as stream:
            self.config = yaml.safe_load(stream)

        ### Settings ###
        self.settings_msg = LarcSettings()
        self.init_msg()
        self.init_gui()
        self.update_gui()

        ## ROS
        rospy.init_node('larc_settings_gui')
        self.config_pub = rospy.Publisher("/larc_settings", LarcSettings, queue_size = 1)
    
        ## Connections
        self.sld_max_hsv_blue_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_blue_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_blue_v.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_blue_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_blue_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_blue_v.valueChanged.connect(self.hsv_value_changed)

        self.sld_max_hsv_green_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_green_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_green_v.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_green_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_green_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_green_v.valueChanged.connect(self.hsv_value_changed)

        self.sld_max_hsv_red_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_red_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_red_v.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_red_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_red_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_red_v.valueChanged.connect(self.hsv_value_changed)

        self.sld_max_hsv_black_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_black_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_max_hsv_black_v.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_black_h.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_black_s.valueChanged.connect(self.hsv_value_changed)
        self.sld_min_hsv_black_v.valueChanged.connect(self.hsv_value_changed)

        self.btn_publish.clicked.connect(self.btn_publish_clicked)
        self.btn_save.clicked.connect(self.btn_save_clicked)

    def hsv_value_changed(self):
        self.update_msg()
        self.update_gui()
        self.btn_publish_clicked()
        

    def btn_publish_clicked(self):
        self.config_pub.publish(self.settings_msg)
    
    def btn_save_clicked(self):
        self.update_config()
        with open(self.config_path, 'w') as file:
            yaml.dump(self.config, file)
        self.btn_publish_clicked()
    
    def update_config(self):
        self.config['max_hsv_blue'] = self.settings_msg.max_hsv_blue
        self.config['min_hsv_blue'] = self.settings_msg.min_hsv_blue
        self.config['max_hsv_green'] = self.settings_msg.max_hsv_green
        self.config['min_hsv_green'] = self.settings_msg.min_hsv_green
        self.config['max_hsv_red'] = self.settings_msg.max_hsv_red
        self.config['min_hsv_red'] = self.settings_msg.min_hsv_red
        self.config['max_hsv_black'] = self.settings_msg.max_hsv_black
        self.config['min_hsv_black'] = self.settings_msg.min_hsv_black

    def init_msg(self):
        self.settings_msg.max_hsv_blue = self.config['max_hsv_blue']
        self.settings_msg.min_hsv_blue = self.config['min_hsv_blue']
        self.settings_msg.max_hsv_green = self.config['max_hsv_green']
        self.settings_msg.min_hsv_green = self.config['min_hsv_green']
        self.settings_msg.max_hsv_red = self.config['max_hsv_red']
        self.settings_msg.min_hsv_red = self.config['min_hsv_red']
        self.settings_msg.max_hsv_black = self.config['max_hsv_black']
        self.settings_msg.min_hsv_black = self.config['min_hsv_black']
    
    def update_gui(self):
        self.lbl_max_hsv_blue_h.setText( 'H: {}'.format(self.sld_max_hsv_blue_h.value()) )
        self.lbl_max_hsv_blue_s.setText( 'S: {}'.format(self.sld_max_hsv_blue_s.value()) )
        self.lbl_max_hsv_blue_v.setText( 'V: {}'.format(self.sld_max_hsv_blue_v.value()) )
        self.lbl_min_hsv_blue_h.setText( 'H: {}'.format(self.sld_min_hsv_blue_h.value()) )
        self.lbl_min_hsv_blue_s.setText( 'S: {}'.format(self.sld_min_hsv_blue_s.value()) )
        self.lbl_min_hsv_blue_v.setText( 'V: {}'.format(self.sld_min_hsv_blue_v.value()) )

        self.lbl_max_hsv_green_h.setText( 'H: {}'.format(self.sld_max_hsv_green_h.value()) )
        self.lbl_max_hsv_green_s.setText( 'S: {}'.format(self.sld_max_hsv_green_s.value()) )
        self.lbl_max_hsv_green_v.setText( 'V: {}'.format(self.sld_max_hsv_green_v.value()) )
        self.lbl_min_hsv_green_h.setText( 'H: {}'.format(self.sld_min_hsv_green_h.value()) )
        self.lbl_min_hsv_green_s.setText( 'S: {}'.format(self.sld_min_hsv_green_s.value()) )
        self.lbl_min_hsv_green_v.setText( 'V: {}'.format(self.sld_min_hsv_green_v.value()) )

        self.lbl_max_hsv_red_h.setText( 'H: {}'.format(self.sld_max_hsv_red_h.value()) )
        self.lbl_max_hsv_red_s.setText( 'S: {}'.format(self.sld_max_hsv_red_s.value()) )
        self.lbl_max_hsv_red_v.setText( 'V: {}'.format(self.sld_max_hsv_red_v.value()) )
        self.lbl_min_hsv_red_h.setText( 'H: {}'.format(self.sld_min_hsv_red_h.value()) )
        self.lbl_min_hsv_red_s.setText( 'S: {}'.format(self.sld_min_hsv_red_s.value()) )
        self.lbl_min_hsv_red_v.setText( 'V: {}'.format(self.sld_min_hsv_red_v.value()) )

        self.lbl_max_hsv_black_h.setText( 'H: {}'.format(self.sld_max_hsv_black_h.value()) )
        self.lbl_max_hsv_black_s.setText( 'S: {}'.format(self.sld_max_hsv_black_s.value()) )
        self.lbl_max_hsv_black_v.setText( 'V: {}'.format(self.sld_max_hsv_black_v.value()) )
        self.lbl_min_hsv_black_h.setText( 'H: {}'.format(self.sld_min_hsv_black_h.value()) )
        self.lbl_min_hsv_black_s.setText( 'S: {}'.format(self.sld_min_hsv_black_s.value()) )
        self.lbl_min_hsv_black_v.setText( 'V: {}'.format(self.sld_min_hsv_black_v.value()) )

    def update_msg(self):
        self.settings_msg.max_hsv_blue[0] = self.sld_max_hsv_blue_h.value()
        self.settings_msg.max_hsv_blue[1] = self.sld_max_hsv_blue_s.value()
        self.settings_msg.max_hsv_blue[2] = self.sld_max_hsv_blue_v.value()
        self.settings_msg.min_hsv_blue[0] = self.sld_min_hsv_blue_h.value()
        self.settings_msg.min_hsv_blue[1] = self.sld_min_hsv_blue_s.value()
        self.settings_msg.min_hsv_blue[2] = self.sld_min_hsv_blue_v.value()
        
        self.settings_msg.max_hsv_green[0] = self.sld_max_hsv_green_h.value()
        self.settings_msg.max_hsv_green[1] = self.sld_max_hsv_green_s.value()
        self.settings_msg.max_hsv_green[2] = self.sld_max_hsv_green_v.value()
        self.settings_msg.min_hsv_green[0] = self.sld_min_hsv_green_h.value()
        self.settings_msg.min_hsv_green[1] = self.sld_min_hsv_green_s.value()
        self.settings_msg.min_hsv_green[2] = self.sld_min_hsv_green_v.value()

        self.settings_msg.max_hsv_red[0] = self.sld_max_hsv_red_h.value()
        self.settings_msg.max_hsv_red[1] = self.sld_max_hsv_red_s.value()
        self.settings_msg.max_hsv_red[2] = self.sld_max_hsv_red_v.value()
        self.settings_msg.min_hsv_red[0] = self.sld_min_hsv_red_h.value()
        self.settings_msg.min_hsv_red[1] = self.sld_min_hsv_red_s.value()
        self.settings_msg.min_hsv_red[2] = self.sld_min_hsv_red_v.value()

        self.settings_msg.max_hsv_black[0] = self.sld_max_hsv_black_h.value()
        self.settings_msg.max_hsv_black[1] = self.sld_max_hsv_black_s.value()
        self.settings_msg.max_hsv_black[2] = self.sld_max_hsv_black_v.value()
        self.settings_msg.min_hsv_black[0] = self.sld_min_hsv_black_h.value()
        self.settings_msg.min_hsv_black[1] = self.sld_min_hsv_black_s.value()
        self.settings_msg.min_hsv_black[2] = self.sld_min_hsv_black_v.value()

    def init_gui(self):
        self.sld_max_hsv_blue_h.setValue( self.settings_msg.max_hsv_blue[0] )
        self.sld_max_hsv_blue_s.setValue( self.settings_msg.max_hsv_blue[1] )
        self.sld_max_hsv_blue_v.setValue( self.settings_msg.max_hsv_blue[2] )
        self.sld_min_hsv_blue_h.setValue( self.settings_msg.min_hsv_blue[0] )
        self.sld_min_hsv_blue_s.setValue( self.settings_msg.min_hsv_blue[1] )
        self.sld_min_hsv_blue_v.setValue( self.settings_msg.min_hsv_blue[2] )
        
        self.sld_max_hsv_green_h.setValue( self.settings_msg.max_hsv_green[0] )
        self.sld_max_hsv_green_s.setValue( self.settings_msg.max_hsv_green[1] )
        self.sld_max_hsv_green_v.setValue( self.settings_msg.max_hsv_green[2] )
        self.sld_min_hsv_green_h.setValue( self.settings_msg.min_hsv_green[0] )
        self.sld_min_hsv_green_s.setValue( self.settings_msg.min_hsv_green[1] )
        self.sld_min_hsv_green_v.setValue( self.settings_msg.min_hsv_green[2] )

        self.sld_max_hsv_red_h.setValue( self.settings_msg.max_hsv_red[0] )
        self.sld_max_hsv_red_s.setValue( self.settings_msg.max_hsv_red[1] )
        self.sld_max_hsv_red_v.setValue( self.settings_msg.max_hsv_red[2] )
        self.sld_min_hsv_red_h.setValue( self.settings_msg.min_hsv_red[0] )
        self.sld_min_hsv_red_s.setValue( self.settings_msg.min_hsv_red[1] )
        self.sld_min_hsv_red_v.setValue( self.settings_msg.min_hsv_red[2] )

        self.sld_max_hsv_black_h.setValue( self.settings_msg.max_hsv_black[0] )
        self.sld_max_hsv_black_s.setValue( self.settings_msg.max_hsv_black[1] )
        self.sld_max_hsv_black_v.setValue( self.settings_msg.max_hsv_black[2] )
        self.sld_min_hsv_black_h.setValue( self.settings_msg.min_hsv_black[0] )
        self.sld_min_hsv_black_s.setValue( self.settings_msg.min_hsv_black[1] )
        self.sld_min_hsv_black_v.setValue( self.settings_msg.min_hsv_black[2] )


app = QApplication(sys.argv)
ui = LarcSettingsGui()
ui.setWindowTitle('LARC - GUI Configuration')
ui.show()   
sys.exit(app.exec_())