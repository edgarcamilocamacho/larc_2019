#!/usr/bin/env python
import rospy
from larc_vision.msg import LarcVisionInfo
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import JointState as JointStateDxl

from fsm import Fsm

fsm = Fsm()
joint_state = None

is_moving_dict = {  'rotating_base': False, 'zipper': False, 'shoulder': False, 
                    'elbow': False, 'wrist_x': False, 'wrist_y': False}

is_moving = False

def vision_callback(data):
    fsm.tick(data, joint_state, is_moving)
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.frame)

def sub_joint_states_callback(msg):
    global joint_state
    joint_state = msg

def is_moving_callback(msg):
    global is_moving, is_moving_dict
    # print(msg.name + ': ' + str(msg.is_moving))
    is_moving_dict[msg.name] = msg.is_moving
    is_moving = False
    for joint in is_moving_dict:
        is_moving = is_moving or is_moving_dict[joint]
    # print(is_moving_dict)
    # print(is_moving)

rospy.init_node('larc_fsm', anonymous=True)
rospy.Subscriber("/larc_vision_info", LarcVisionInfo, vision_callback)
rospy.Subscriber('/joint_states', JointState, sub_joint_states_callback)

rospy.Subscriber('/rotating_base/state', JointStateDxl, is_moving_callback)
rospy.Subscriber('/zipper/state', JointStateDxl, is_moving_callback)
rospy.Subscriber('/shoulder/state', JointStateDxl, is_moving_callback)
rospy.Subscriber('/elbow/state', JointStateDxl, is_moving_callback)
rospy.Subscriber('/wrist_x/state', JointStateDxl, is_moving_callback)
rospy.Subscriber('/wrist_y/state', JointStateDxl, is_moving_callback)

# rospy.Subscriber("/larc_movement/is_moving", Bool, is_moving_callback)
rospy.spin()
