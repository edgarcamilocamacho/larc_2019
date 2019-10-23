#!/usr/bin/env python
# license removed for brevity
import rospy
import rospkg
import yaml
from std_msgs.msg import String, Bool
from larc_settings.msg import LarcSettings

GUI_ACTIVE = False
settings_msg = None

rospack = rospkg.RosPack()
larc_settings_path = rospack.get_path('larc_settings')
config_path = larc_settings_path + '/config/config.yaml'

def create_msg():
    global settings_msg
    with open(config_path, 'r') as stream:
        config = yaml.safe_load(stream)
    settings_msg = LarcSettings()
    settings_msg.max_hsv_blue = config['max_hsv_blue']
    settings_msg.min_hsv_blue = config['min_hsv_blue']
    settings_msg.max_hsv_green = config['max_hsv_green']
    settings_msg.min_hsv_green = config['min_hsv_green']
    settings_msg.max_hsv_red = config['max_hsv_red']
    settings_msg.min_hsv_red = config['min_hsv_red']
    settings_msg.max_hsv_black = config['max_hsv_black']
    settings_msg.min_hsv_black = config['min_hsv_black']
    settings_msg.ref_x = config['ref_x']
    settings_msg.ref_y = config['ref_y']
    settings_msg.ref_s = config['ref_s']

def gui_active_callback(data):
    global GUI_ACTIVE
    GUI_ACTIVE = data.data
    create_msg()

rospy.init_node('larc_settings_node', anonymous=True)
config_pub = rospy.Publisher("/larc_settings", LarcSettings, queue_size = 1)
rospy.Subscriber('/larc/settings/gui_active', Bool, gui_active_callback)
create_msg()

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    if not GUI_ACTIVE:
        config_pub.publish(settings_msg)
    rate.sleep()
