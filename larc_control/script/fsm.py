import rospy
# from periphery import GPIO

from motors_bridge import MotorsBridge
from PID import PID
import math
import time

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray, Float64, String

BTN1_PIN = 432

SRC_TIME = 30

class Fsm:

    def __init__(self):
        self.state = self.s_start_state
        self.state_1 = None

        ## PID
        self.pid_w = PID(4.0, 0.0, 0.0)
        self.pid_w.SetPoint = 0.0
        self.pid_w.setWindup(20.0)
        self.meas_w_1 = 0

        self.pid_x = PID(4.0, 0.0, 0.0)
        self.pid_x.SetPoint = 0.0
        self.pid_x.setWindup(20.0)
        self.meas_x_1 = 0

        self.pid_y = PID(4.0, 0.0, 0.0)
        self.pid_y.SetPoint = 0.0
        self.pid_y.setWindup(20.0)
        self.meas_y_1 = 0
        
        ## Variables
        self.timer1 = 0
        self.count_zero = 0

        ## ROS
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Vector3, queue_size = 1)
        self.pub_move = rospy.Publisher("/larc_movement/go_to_pos", Float32MultiArray, queue_size = 1)
        self.pub_gripper = rospy.Publisher("/gripper/command", Float64, queue_size = 1)

        self.msg = Float32MultiArray()

        self.ret_state = []

        self.pick_phases = []

        ## Others
        self.mb = MotorsBridge()
        # self.button1 = GPIO(BTN1_PIN, "in")
        # assert gpio.pin == BTN1_PIN
        # assert gpio.fd > 0

    def tick(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            print('Entering to state "{}"'.format(self.state.__name__))
        # print('Entering to state "{}"'.format(self.state.__name__))
        next_state = self.state(vision_info, joint_state, is_moving)
        self.state_1 = self.state
        self.state = next_state
    
    def s_start_state(self, vision_info, joint_state, is_moving):
        self.timer1 = 0
        return self.s_wait_button

    def s_wait_button(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.mb.go_to_position(['rotating_base'], [0.0])
        self.timer1 += 1
        if self.timer1 < 50:
            return self.s_wait_button
        else:
            return self.init_position
        
    def init_position(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.msg.data = [0.0, 0.0, 0.1, 1.2, 0.4, 1.57] 
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<20 or is_moving:
            return self.init_position
        else:    
            return self.s_forward1
    
    def s_forward1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_cmd_vel.publish(0.0, 5.0, 0.0)
        if vision_info.black_strip_flag and vision_info.black_strip_y>50:
            return self.s_forward2
        else:
            return self.s_forward1

    def s_forward2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_cmd_vel.publish(0.0, 2.0, 0.0)
        if vision_info.black_strip_flag and vision_info.black_strip_y>200:
            return self.s_forward3
        else:
            return self.s_forward2
        
    def s_forward3(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_cmd_vel.publish(0.0, 1.5, 0.0)
        if vision_info.num_containers==0:
            return self.s_forward3
        else:
            return self.s_wait2

    def s_wait2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<5:
            return self.s_wait2
        else:
            if vision_info.num_containers==2:
                self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
                return self.s_control1
            else:
                if len(vision_info.cont_x)==0:
                    return self.s_wait2
                elif vision_info.cont_x[0] > 320:
                    return self.s_right
                else:
                    return self.s_left

    def s_right(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_cmd_vel.publish(2.0, 0.0, 0.0)
        if vision_info.num_containers<2:
            return self.s_right
        else:
            return self.s_wait1
    
    def s_left(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_cmd_vel.publish(-2.0, 0.0, 0.0)
        if vision_info.num_containers<2:
            return self.s_left
        else:
            return self.s_wait1
    
    def s_wait1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<10:
            return self.s_wait1
        else:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
            return self.s_control1

    def s_control1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_gripper.publish(0.65)
        meas_w = vision_info.error_s
        if math.isnan(meas_w):
            meas_w = self.meas_w_1
        if meas_w>0.3:
            meas_w = 0.3
        if meas_w<-0.3:
            meas_w = -0.3
        self.pid_w.update(meas_w)
        cw = -self.pid_w.output
        # print('meas_w: ' + str(meas_w) + ', cw: ' + str(cw))
        self.pub_cmd_vel.publish(0.0, 0.0, cw)
        self.meas_w_1 = meas_w
        if meas_w>=0.01:
            return self.s_control1
        else:
            return self.s_control2

    def s_control2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.count_zero = 0
        ###
        meas_w = vision_info.error_s
        if math.isnan(meas_w):
            meas_w = self.meas_w_1
        if meas_w>0.3:
            meas_w = 0.3
        if meas_w<-0.3:
            meas_w = -0.3
        self.pid_w.update(meas_w)
        cw = -self.pid_w.output
        self.meas_w_1 = meas_w
        ###
        meas_x = vision_info.error_x/640.0
        if math.isnan(meas_x):
            meas_x = self.meas_x_1
        if meas_x>0.3:
            meas_x = 0.3
        if meas_x<-0.3:
            meas_x = -0.3
        self.pid_x.update(meas_x)
        cx = self.pid_x.output
        self.meas_x_1 = meas_x
        ###
        meas_y = vision_info.error_y/640.0
        if math.isnan(meas_y):
            meas_y = self.meas_y_1
        if meas_y>0.3:
            meas_y = 0.3
        if meas_y<-0.3:
            meas_y = -0.3
        self.pid_y.update(meas_y)
        cy = -self.pid_y.output
        self.meas_y_1 = meas_y

        if abs(meas_w)<0.01 and abs(meas_x)<0.01 and abs(meas_y)<0.01:
            self.count_zero += 1
        else:
            self.count_zero = 0

        # print('meas_w: ' + str(meas_w) + ', cw: ' + str(cw))
        # print('meas_x: ' + str(meas_x) + ', cx: ' + str(cx))
        # print('meas_y: ' + str(meas_y) + ', cy: ' + str(cy))
        
        if self.count_zero<=15:
            self.pub_cmd_vel.publish(cx, cy, cw)
            return self.s_control2
        else:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
            return self.dummy_1
        

    ## DUMMY
    def dummy_1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.65)
        self.pick_phases = [    [-0.15, 0.055, -0.5, 1.5, -0.5, 1.43],
                                [-0.12, 0.055, -1.0, 1.0, -0.5, 1.43],
                                [-0.15, 0.055, -0.5, 1.5, -0.5, 1.43] ]
        self.ret_state.append(self.dummy_2)
        return self.s_pick_0

    def dummy_2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [2.1, 0.08, -0.5, 1.5, -0.5, 0.55],
                                [2.2, 0.08, -1.495, 0.9, -0.895, 0.65],
                                [2.16, 0.08, -1.495, 0.9, -0.895, 0.67],
                                [2.1, 0.08, -0.5, 1.5, -0.5, 0.55] ]
        self.ret_state.append(self.dummy_3)
        return self.s_release_0

    def dummy_3(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [-0.23, 0.04, -0.5, 1.5, -0.5, -1.80],
                                [-0.21, 0.04, -1.0, 1.0, -0.5, -1.80],
                                [-0.23, 0.04, -0.5, 1.5, -0.5, -1.80] ]
        self.ret_state.append(self.dummy_4)
        return self.s_pick_0

    def dummy_4(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [2.1, 0.08, -0.5, 1.5, -0.5, 0.55],
                                [2.20, 0.088, -1.22, 1.15, -0.82, 0.64],
                                [2.16, 0.088, -1.22, 1.15, -0.82, 0.64],
                                [2.1, 0.08, -0.5, 1.5, -0.5, 0.55] ]
        self.ret_state.append(self.dummy_5)
        return self.s_release_0

    def dummy_5(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [-0.15, 0.055, -0.5, 1.5, -0.5, 1.43],
                                [-0.14, 0.042, -1.25, 0.81, -0.55, 1.43],
                                [-0.15, 0.055, -0.5, 1.5, -0.5, 1.43] ]
        self.ret_state.append(self.dummy_6)
        return self.s_pick_0

    def dummy_6(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [2.1, 0.08, -0.5, 1.5, -0.5, 0.55],
                                [2.19, 0.08, -1.11, 1.05, -0.53, 0.64],
                                [2.15, 0.08, -1.11, 1.05, -0.53, 0.64],
                                [2.1, 0.08, -0.5, 1.5, -0.5, 0.55] ]
        self.ret_state.append(self.dummy_7)
        return self.s_release_0

    def dummy_7(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [-0.23, 0.04, -0.5, 1.5, -0.5, -1.80],
                                [-0.21, 0.04, -1.25, 0.81, -0.55, -1.80],
                                [-0.23, 0.04, -0.5, 1.5, -0.5, -1.80] ]
        self.ret_state.append(self.dummy_8)
        return self.s_pick_0

    def dummy_8(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            pass
        self.pick_phases = [    [2.1, 0.08, -0.5, 1.5, -0.5, 0.55],
                                [2.19, 0.08, -0.92, 1.22, -0.58, 0.64],
                                [2.13, 0.08, -0.92, 1.22, -0.58, 0.64],
                                [2.1, 0.08, -0.5, 1.5, -0.5, 0.55] ]
        self.ret_state.append(self.s_end_state)
        return self.s_release_0

    ## CONTENEDORES

    def s_close_gripper(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.27)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<10:
            return self.s_close_gripper
        else:    
            return self.ret_state.pop()
    
    def s_open_gripper(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.35)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<2:
            return self.s_open_gripper
        else:    
            return self.s_open_gripper_1
    
    def s_open_gripper_1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.40)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<2:
            return self.s_open_gripper_1
        else:    
            return self.s_open_gripper_3
    
    def s_open_gripper_2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.45)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<2:
            return self.s_open_gripper_2
        else:    
            return self.s_open_gripper_3
    
    def s_open_gripper_3(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.65)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<4:
            return self.s_open_gripper_3
        else:    
            return self.ret_state.pop()

    # pick
    def s_pick_0(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[0]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_pick_0
        else:    
            return self.s_pick_1
    def s_pick_1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[1]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_pick_1
        else:    
            self.ret_state.append(self.s_pick_2)
            return self.s_close_gripper
    def s_pick_2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[2]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_pick_2
        else:    
            return self.ret_state.pop()
    
    # release
    def s_release_0(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[0]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_release_0
        else:    
            return self.s_release_1
    def s_release_1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[1]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_release_1
        else:    
            self.ret_state.append(self.s_release_2)
            return self.s_open_gripper
    def s_release_2(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[2]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<5 or is_moving:
            return self.s_release_2
        else:    
            return self.s_release_3
    def s_release_3(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.pick_phases[3]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_release_3
        else:    
            return self.ret_state.pop()
    
    def s_end_state(self, vision_info, joint_state, is_moving):
        return self.s_end_state