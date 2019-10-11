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

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import RobotState
from std_msgs.msg import Float64, String

node_name = 'poppy_control_moveit'
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

rospy.loginfo(rospy.get_caller_id() + ' Ready!')

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
    execute_plan( plan_to_joint_values( jv ) )

# while(True):
#     go_to_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#     time.sleep(4)
#     go_to_pos([0.0, 0.1, -0.3, 1.1, 0.1, 1.57])
#     time.sleep(4)

# go_to_pos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
# go_to_pos([0.0, 0.1, -0.3, 1.1, 0.1, 1.57])

# jv = [0.0, 0.1, -0.3, 1.1, 0.1, 1.57]
# jv = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# jv = [0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
# execute_plan( plan_to_joint_values( jv ) )

# pose = current_pose()
# pose.position.x += 0.01
# plan = plan_to_pose(pose)
# execute_plan(plan)

# pub_gripper.publish(0.2)   