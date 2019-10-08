#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

rospy.init_node("initial_position")

zipper = rospy.Publisher("/zipper/command", Float64, queue_size = 1)
wrist_y = rospy.Publisher("/wrist_y/command", Float64, queue_size = 1)
wrist_x = rospy.Publisher("/wrist_x/command", Float64, queue_size = 1)
elbow = rospy.Publisher("/elbow/command", Float64, queue_size = 1)
gripper = rospy.Publisher("/gripper/command", Float64, queue_size = 1)
base = rospy.Publisher("/base/command", Float64, queue_size = 1)
shoulder = rospy.Publisher("/shoulder/command", Float64, queue_size = 1)

time.sleep(0.5)

zipper.publish(-2.1)
wrist_y.publish(0.0) 
wrist_x.publish(0.3) # 0.3 -> 1.2
elbow.publish(0.0)
# gripper.publish(0.0)
base.publish(0.7)
shoulder.publish(0)