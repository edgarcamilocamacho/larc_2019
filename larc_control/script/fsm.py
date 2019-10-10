import rospy
from periphery import GPIO

from motors_bridge import MotorsBridge

from geometry_msgs.msg import Vector3

BTN1_PIN = 432

class Fsm:

    def __init__(self):
        self.state = self.s_start_state
        self.state_1 = None

        ## Variables
        self.timer1 = 0

        ## ROS
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Vector3, queue_size = 1)

        ## Others
        self.mb = MotorsBridge()
        # self.button1 = GPIO(BTN1_PIN, "in")
        # assert gpio.pin == BTN1_PIN
        # assert gpio.fd > 0

    def tick(self, vision_info, joint_state):
        if self.state != self.state_1:
            print('Entering to state "{}"'.format(self.state.__name__))
        next_state = self.state(vision_info, joint_state)
        self.state_1 = self.state
        self.state = next_state
    
    def s_start_state(self, vision_info, joint_state):
        self.timer1 = 0
        return self.s_wait_button

    def s_wait_button(self, vision_info, joint_state):
        if self.state != self.state_1:
            self.mb.go_to_position(['rotating_base'], [0.0])
        self.timer1 += 1
        if self.timer1 < 50:
            return self.s_wait_button
        else:
            return self.s_init1
    
    def s_init1(self, vision_info, joint_state):
        if self.state != self.state_1:
            # self.mb.plan_to_angles(['shoulder'], [-0.48], joint_state, 60)
            self.mb.plan_to_angles(['shoulder'], [0.50], joint_state, 30)
        if self.mb.plan_step():
            return self.s_init2
        else:
            return self.s_init1
    
    def s_init2(self, vision_info, joint_state):
        if self.state != self.state_1:
            self.mb.plan_to_angles(
                ['shoulder', 'elbow', 'wrist_x', 'wrist_y', 'gripper'], 
                [-0.48, 1.05, -0.15, 0.00, 0.50], 
                joint_state, 
                60)
        if self.mb.plan_step():
            return self.s_forward1
        else:
            return self.s_init2
    
    def s_forward1(self, vision_info, joint_state):
        if self.state != self.state_1:
            self.pub_cmd_vel.publish(0.0, 3.0, 0.0)
        if vision_info.black_strip_flag and vision_info.black_strip_y>200:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
            return self.s_end_state
        else:
            return self.s_forward1

    def s_end_state(self, vision_info, joint_state):
        return self.s_end_state