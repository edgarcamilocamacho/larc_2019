#!/usr/bin/env python

from geometry_msgs.msg import Vector3
import rospy
from std_msgs.msg import Float64
from dynamixel_controllers.srv import TorqueEnable
from time import sleep
import pygame

JOY_INDEX = {
	'A':0, 'B':1, 'X':2, 'Y':3, 'LB':4, 'RB':5, 'BACK':6, 'START':7, 'XBOX':8, 'LS':9, 'RS':10
}
AXIS_INDEX = {
	'LSH':0, 'LSV':1, 'LT':2, 'RSH':3, 'RSV':4, 'RT':5
}

rospy.init_node("joystick_controller")


#sub = rospy.Subscriber("/cmd_vel", Vector3, transform_) #Toca mirar la lectura

vel = rospy.Publisher("/cmd_vel", Vector3, queue_size = 1)
base = rospy.Publisher("/base/command", Float64, queue_size = 1)
zipper = rospy.Publisher("/zipper/command", Float64, queue_size = 1)
shoulder = rospy.Publisher("/shoulder/command", Float64, queue_size = 1)
elbow = rospy.Publisher("/elbow/command", Float64, queue_size = 1)
wrist_x = rospy.Publisher("/wrist_x/command", Float64, queue_size = 1)
wrist_y = rospy.Publisher("/wrist_y/command", Float64, queue_size = 1)
gripper = rospy.Publisher("/gripper/command", Float64, queue_size = 1)

joints = [base, zipper, shoulder, elbow, wrist_x, wrist_y, gripper]

pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()

if joystick_count==0:
	print('Connect the Joystick')
	exit()
joystick = pygame.joystick.Joystick(0)
joystick.init()
joystick_num_buttons = joystick.get_numbuttons()

axes = joystick.get_numaxes()

def get_joystick_state():
	joystick_buttons = [0]*joystick_num_buttons
	for i in range(joystick_num_buttons):
		joystick_buttons[i] = joystick.get_button( i )
	return joystick_buttons

def get_axis_state():
	joystick_axis = [0]*axes
	for i in range(axes):
		joystick_axis[i] = joystick.get_axis( i )
	return joystick_axis

def control_torque(active, joints):
	#for i, j in range(joints)
	#torque[i] = rospy.ServiceProxy('service_name', my_package.srv.Foo)


pygame.event.get()
keys = get_joystick_state()

while(keys[JOY_INDEX['XBOX']] == 0):
	pygame.event.get()
	keys = get_joystick_state()
	axis = get_axis_state()

	#Wheels
	vel.publish(axis[AXIS_INDEX['LSV']]*3, axis[AXIS_INDEX['LSH']]*3, 0)
	#shoulder.publish(axis[AXIS_INDEX['RSV']]*3)
	#base.publish(axis[AXIS_INDEX['RSH']]*3)

	#Arm_another_joint
	# if keys[JOY_INDEX['Y']]==1:
	# 	dxl.setAngle(Elbowjoint, dxl.motors["joint2_0"]['robotAngle']+10)
	# elif keys[JOY_INDEX['A']]==1 and keys[JOY_INDEX['RS']]==0 and keys[JOY_INDEX['LS']]==0:
	# 	dxl.setAngle(Elbowjoint, dxl.motors["joint2_0"]['robotAngle']-10)
	# elif keys[JOY_INDEX['X']]==1:
	# 	dxl.setAngle(Wristjoint, dxl.motors["joint3"]['robotAngle']+10)
	# elif keys[JOY_INDEX['B']]==1:
	# 	dxl.setAngle(Wristjoint, dxl.motors["joint3"]['robotAngle']-10)

	# if keys[JOY_INDEX['RB']]==1:
	# 	dxl.setAngle(Handjoint, dxl.motors["joint4"]['robotAngle']+10)
	# elif keys[JOY_INDEX['LB']]==1:
	# 	dxl.setAngle(Handjoint, dxl.motors["joint4"]['robotAngle']-10)
	# elif axis[AXIS_INDEX['RT']]>-1:
	# 	dxl.setAngle(grippe, dxl.motors['pinz']['robotAngle']+5*(axis[AXIS_INDEX['RT']]+1))
	# elif axis[AXIS_INDEX['LT']]>-1:
	# 	dxl.setAngle(grippe, dxl.motors['pinz']['robotAngle']-5*(axis[AXIS_INDEX['LT']]+1))

	# if keys[JOY_INDEX['RS']]==1:
	# 	dxl.setAngle(zipper, dxl.motors["zipper1"]['robotAngle']-20)
	# elif keys[JOY_INDEX['LS']]==1:
	# 	dxl.setAngle(zipper, dxl.motors["zipper1"]['robotAngle']+20)

	sleep(0.01)

print('Bis bald')
dxl.stop()
pygame.joystick.quit()
