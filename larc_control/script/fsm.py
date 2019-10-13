import rospy
# from periphery import GPIO

from motors_bridge import MotorsBridge
from PID import PID
import math
import time

from larc_movement import go_to_pos

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray

BTN1_PIN = 432

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

        self.msg = Float32MultiArray()

        ## Others
        self.mb = MotorsBridge()
        # self.button1 = GPIO(BTN1_PIN, "in")
        # assert gpio.pin == BTN1_PIN
        # assert gpio.fd > 0

    def tick(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            print('Entering to state "{}"'.format(self.state.__name__))
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
            self.msg.data = [0.0, 0.0, 0.1, 1.2, 0.4, 1.57] 
            self.pub_move.publish(self.msg)
            ####### wait
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
        if vision_info.num_containers==0:
            return self.s_forward2
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
            pass
        meas_w = vision_info.cont_slope
        if math.isnan(meas_w):
            meas_w = self.meas_w_1
        if meas_w>0.3:
            meas_w = 0.3
        if meas_w<-0.3:
            meas_w = -0.3
        self.pid_w.update(meas_w)
        cw = self.pid_w.output
        print('meas_w: ' + str(meas_w) + ', cw: ' + str(cw))
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
        meas_w = vision_info.cont_slope
        if math.isnan(meas_w):
            meas_w = self.meas_w_1
        if meas_w>0.3:
            meas_w = 0.3
        if meas_w<-0.3:
            meas_w = -0.3
        self.pid_w.update(meas_w)
        cw = self.pid_w.output
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

        print('meas_w: ' + str(meas_w) + ', cw: ' + str(cw))
        print('meas_x: ' + str(meas_x) + ', cx: ' + str(cx))
        print('meas_y: ' + str(meas_y) + ', cy: ' + str(cy))
        
        if self.count_zero>10:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
            return self.s_end_state
        else:
            self.pub_cmd_vel.publish(cx, cy, cw)
            return self.s_control2

    def s_end_state(self, vision_info, joint_state, is_moving):
        return self.s_end_state