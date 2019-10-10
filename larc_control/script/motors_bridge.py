import numpy as np

import rospy

from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from dynamixel_controllers.srv import TorqueEnable

class MotorsBridge:

    def __init__(self):
        self.pubs = {
            'cmd_vel': rospy.Publisher("/cmd_vel", Vector3, queue_size = 1),
            'rotating_base': rospy.Publisher("/rotating_base/command", Float64, queue_size = 1),
            'zipper': rospy.Publisher("/zipper/command", Float64, queue_size = 1),
            'shoulder': rospy.Publisher("/shoulder/command", Float64, queue_size = 1),
            'elbow': rospy.Publisher("/elbow/command", Float64, queue_size = 1),
            'wrist_x': rospy.Publisher("/wrist_x/command", Float64, queue_size = 1),
            'wrist_y': rospy.Publisher("/wrist_y/command", Float64, queue_size = 1),
            'gripper': rospy.Publisher("/gripper/command", Float64, queue_size = 1),
        }

        ## Plan
        self.plan_trajectories = None
        self.plan_index = 0
        self.plan_frames = 0
    
    def go_to_position(self, joints, pos):
        for i, joint in enumerate(joints):
            self.pubs[joint].publish(pos[i])
    
    def plan_to_angles(self, joints, pos, joint_state, frames):
        self.plan_index = 0
        self.plan_trajectories = {}
        self.plan_frames = frames
        for i, joint in enumerate(joints):
            origin = joint_state.position[joint_state.name.index(joint)]
            target = pos[i]
            self.plan_trajectories[joint] = np.linspace(origin, target, frames)
    
    def plan_step(self):
        for joint in self.plan_trajectories:
            self.pubs[joint].publish( self.plan_trajectories[joint][self.plan_index] )
        self.plan_index += 1
        if self.plan_index >= self.plan_frames:
            return True
        else:
            return False
