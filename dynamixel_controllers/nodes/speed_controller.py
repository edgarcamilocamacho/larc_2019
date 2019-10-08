#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3


Vx = 0.0
Vy = 0.0
VW = 0.0

V1 = 0.0
V2 = 0.0
V3 = 0.0
V4 = 0.0

send = True

def transform_(msg):    
    global Vx
    global Vy
    global Vw

    global V1
    global V2
    global V3
    global V4
    global send

    Vx = msg.x
    Vy = msg.y
    Vw = msg.z

    V3 = Vy + Vw  #V1
    V2 = -Vx + Vw #V2
    V4 = -Vy + Vw #V3
    V1 = Vx + Vw #V4

    send = False

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/cmd_vel", Vector3, transform_)

wheel1 = rospy.Publisher("/wheel1/command", Float64, queue_size = 1)
wheel2 = rospy.Publisher("/wheel2/command", Float64, queue_size = 1)
wheel3 = rospy.Publisher("/wheel3/command", Float64, queue_size = 1)
wheel4 = rospy.Publisher("/wheel4/command", Float64, queue_size = 1)
r = rospy.Rate(4)
while not rospy.is_shutdown():

    #while not send:
        send = True
        wheel1.publish(V1)
        wheel2.publish(V2)
        wheel3.publish(V3)
        wheel4.publish(V4)
        r.sleep()