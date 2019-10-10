#!/usr/bin/env python
import rospy
from larc_vision.msg import LarcVisionInfo
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from fsm import Fsm

fsm = Fsm()
joint_state = None

def vision_callback(data):
    fsm.tick(data, joint_state)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.frame)

def sub_joint_states_callback(msg):
    global joint_state
    joint_state = msg

rospy.init_node('larc_fsm', anonymous=True)
rospy.Subscriber("/larc_vision_info", LarcVisionInfo, vision_callback)
rospy.Subscriber('/joint_states', JointState, sub_joint_states_callback)
rospy.spin()
