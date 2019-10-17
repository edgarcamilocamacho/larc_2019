#!/usr/bin/env python2
import sys
import os
import rospy
import numpy as np
import time
import math

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotState
from std_msgs.msg import Float64, String

node_name = 'larc_movement'
group_id = 'all'

rospy.loginfo(rospy.get_caller_id() + ' Node created')
rospy.init_node(node_name, anonymous=False)
rospy.loginfo(rospy.get_caller_id() + ' Connecting with moveit...')
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander(group_id)
group.set_planner_id("RRTConnectkConfigDefault")
group.set_planner_id(group_id)
group.set_planning_time(5)

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
                                               moveit_msgs.msg.DisplayTrajectory, \
                                               queue_size=20)

pub_gripper = rospy.Publisher("/gripper/command", Float64, queue_size = 1)

# rospy.spin()

def set_current_robot_state():
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    # joint_state.header.stamp.secs += 1
    joint_state.name = group.get_active_joints()
    start_position = group.get_current_joint_values()  
    joint_state.position = start_position    
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    group.set_start_state(moveit_robot_state)

def get_random_joint_values():
    return group.get_random_joint_values()

def get_current_joint_values():
    return group.get_current_joint_values() 

def plan_to_joint_values(joint_values):
    try:
        group.set_joint_value_target(joint_values)
    except:
        rospy.logwarn(rospy.get_caller_id() + ' Incorrect target positions')
        return None
    set_current_robot_state()
    plan = group.plan()
    if len(plan.joint_trajectory.points)>0:
        rospy.loginfo(rospy.get_caller_id() + ' Correct plan')
        return plan
    else:
        rospy.logwarn(rospy.get_caller_id() + ' Could not plan')
        return None

def current_pose():
    return group.get_current_pose().pose

def plan_to_pose(pose):
    try:
        group.set_pose_target(pose)
    except:
        rospy.logwarn(rospy.get_caller_id() + ' Incorrect target pose')
        return None
    set_current_robot_state()
    plan = group.plan()
    if len(plan.joint_trajectory.points)>0:
        rospy.loginfo(rospy.get_caller_id() + ' Correct plan')
        return plan
    else:
        rospy.logwarn(rospy.get_caller_id() + ' Could not plan')
        return None

def execute_plan(plan, wait=True):
    if plan==None:
        rospy.logwarn(rospy.get_caller_id() + ' No plan')
        return 
    group.execute(plan, wait=wait)

def go_to_pos(jv):

    plan = plan_to_joint_values( jv )
    if plan==None:
        rospy.logwarn(rospy.get_caller_id() + ' No plan. plan=None')
        return
    error = 10.0
    while error>0.07:
        if error!=10.0:
            rospy.logwarn(rospy.get_caller_id() + ' Plan not executed, retrying...')
        execute_plan( plan )
        error = np.sqrt(np.sum((get_current_joint_values()-np.array(jv))**2))
    rospy.logwarn(rospy.get_caller_id() + ' Plan executed')

    # error = 10.0
    # while error>0.07:
    #     execute_plan( plan_to_joint_values( jv ) )
    #     error = np.sqrt(np.sum((get_current_joint_values()-np.array(jv))**2))
    #     print('error='+str(error))

def go_to_pos_callback(msg):
    movement_flag_publisher.publish(True)
    rospy.loginfo(rospy.get_caller_id() + ' Moving to {}'.format(msg.data))
    go_to_pos(msg.data)
    movement_flag_publisher.publish(False)

movement_flag_publisher = rospy.Publisher('/larc_movement/is_moving', Bool, queue_size=1)
rospy.Subscriber("/larc_movement/go_to_pos", Float32MultiArray, go_to_pos_callback)

rospy.loginfo(rospy.get_caller_id() + ' Ready!')

rospy.spin()
