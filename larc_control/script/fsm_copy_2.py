import rospy
from periphery import GPIO


from motors_bridge import MotorsBridge
from PID import PID
import math
import time

from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray, Float64, String
import rospkg

rospack = rospkg.RosPack()
var_path = rospack.get_path('larc_control')
var_path = var_path + '/script/var.py'

from var import *
# execfile(var_path)

BTN1_PIN = 431
BTN2_PIN = 432

SRC_TIME = 30

ERROR_W = 0.01
ERROR_X = 0.005
ERROR_Y = 0.005

class Fsm:

    def __init__(self):
        self.state = self.s_start_state
        self.state_1 = None

        ## PID
        self.pid_w = PID(1.0, 0.0, 0.0)
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
        self.ret_from_init_pos = None
        self.ret_from_cam_pos = None
        self.ret_from_control = None

        self.pick_phases = []
        self.release_phases = []

        self.left_color = ''
        self.right_color = ''
        self.ctp = ''

        self.count_pick = 0
        self.count_red = 0
        self.count_green = 0
        self.count_blue = 0

        self.fsm_start_signal = 0

        ## Others
        self.mb = MotorsBridge()
        self.button1 = GPIO(BTN1_PIN, "in")
        assert self.button1.pin == BTN1_PIN
        assert self.button1.fd > 0
        self.button2 = GPIO(BTN2_PIN, "in")
        assert self.button2.pin == BTN2_PIN
        assert self.button2.fd > 0

    def tick(self, vision_info, joint_state, is_moving, fsm_start_signal):
        if self.state != self.state_1:
            rospy.loginfo('Entering to state "{}"'.format(self.state.__name__))
        self.fsm_start_signal = fsm_start_signal
        if self.fsm_start_signal == -1:
            next_state = self.s_idle
        elif not(self.state==self.s_idle or self.state==self.init_position_0 or self.state==self.s_cam_position) and ( (not self.button1.read()) or (not self.button2.read()) ):
            next_state = self.s_stop
        else:
            next_state = self.state(vision_info, joint_state, is_moving)
        self.state_1 = self.state
        self.state = next_state

    def s_stop(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<45:
            return self.s_stop
        else:
            return self.s_idle

    def s_start_state(self, vision_info, joint_state, is_moving):
        self.timer1 = 0
        return self.s_idle

    def s_idle(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_cmd_vel.publish(0.0, 0.0, 0.0)
            self.count_pick = 0
            self.count_red = 0
            self.count_green = 0
            self.count_blue = 0
        self.timer1 += 1
        if self.fsm_start_signal == 1 or (not self.button1.read()):
            self.ret_from_init_pos = self.s_idle
            return self.init_position_0
        elif self.fsm_start_signal == 2:
            self.ret_from_cam_pos = self.s_idle
            return self.s_cam_position
        elif self.fsm_start_signal == 3:
            self.ret_from_control = self.s_idle
            return self.s_control2
        elif self.fsm_start_signal == 4:
            self.ret_from_cam_pos = self.s_forward1
            self.ret_from_control = self.s_idle
            return self.s_cam_position
        elif self.fsm_start_signal == 5:
            return self.choose_1
        elif self.fsm_start_signal == 6 or (not self.button2.read()):
            self.ret_from_cam_pos = self.s_forward1
            self.ret_from_control = self.choose_1
            return self.s_cam_position
        else:
            return self.s_idle

    def init_position_0(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.msg.data = init_positions[0]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<20 or is_moving:
            return self.init_position_0
        else:
            return self.init_position_1

    def init_position_1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.pub_gripper.publish(0.25)
            self.msg.data = init_positions[1]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<20 or is_moving:
            return self.init_position_1
        else:
            return self.ret_from_init_pos

    def s_cam_position(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1: # Recien llego a este estado?
            self.msg.data = cam_pos
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<20 or is_moving:
            return self.s_cam_position
        else:
            return self.ret_from_cam_pos

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
                return self.s_control2
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
            return self.s_control2

    # def s_control1(self, vision_info, joint_state, is_moving):
    #     if self.state != self.state_1: # Recien llego a este estado?
    #         self.pub_gripper.publish(0.65)
    #     meas_w = vision_info.error_s
    #     if math.isnan(meas_w):
    #         meas_w = self.meas_w_1
    #     if meas_w>0.3:
    #         meas_w = 0.3
    #     if meas_w<-0.3:
    #         meas_w = -0.3
    #     self.pid_w.update(meas_w)
    #     cw = -self.pid_w.output
    #     # print('meas_w: ' + str(meas_w) + ', cw: ' + str(cw))
    #     self.pub_cmd_vel.publish(0.0, 0.0, cw)
    #     self.meas_w_1 = meas_w
    #     if meas_w>=0.01:
    #         return self.s_control1
    #     else:
    #         return self.s_control2

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
        # cw = 0.0
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

        # print('')
        # print(meas_w)
        # print(meas_x)
        # print(meas_y)

        if abs(meas_w)<ERROR_W and abs(meas_x)<ERROR_X and abs(meas_y)<ERROR_Y:
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
            return self.ret_from_control

    ## CHOOSE
    def choose_1(self, vision_info, joint_state, is_moving):
        self.pub_gripper.publish(0.65)
        if len(pick_order)>0:
            # self.ctp = pick_order.pop(0)
            self.ctp = pick_order[self.count_pick]
            self.count_pick += 1
            rospy.loginfo('Picking: "{}"'.format(self.ctp))
            if self.ctp[1]=='1':
                self.pick_phases = [    pick_routines[self.ctp][0],
                                        pick_routines[self.ctp][1],
                                        pick_routines[self.ctp][2] ]
            else:
                self.pick_phases = [    pick_routines[self.ctp][0],
                                        pick_routines[self.ctp][1],
                                        pick_routines[self.ctp][0] ]
            return self.s_pick_0
        else:
            return self.s_idle

    def choose_2(self, vision_info, joint_state, is_moving):
        if self.ctp[0]=='0' or self.ctp[0]=='2' or self.ctp[0]=='4':
            rospy.loginfo('Picking the left one, color: {}'.format(self.left_color))
            if self.left_color=='green' and self.count_green>=7:
                routine = release_red_routines[self.count_red]
                self.count_red += 1
            elif self.left_color=='blue' and self.count_blue>=7:
                routine = release_red_routines[self.count_red]
                self.count_red += 1
            elif self.left_color=='green':
                routine = release_green_routines[self.count_green]
                self.count_green += 1
            elif self.left_color=='blue':
                routine = release_blue_routines[self.count_blue]
                self.count_blue += 1
            else:
                routine = release_red_routines[self.count_red]
                self.count_red += 1

        elif self.ctp[0]=='1' or self.ctp[0]=='3' or self.ctp[0]=='5':
            rospy.loginfo('Picking the right one, color: {}'.format(self.right_color))
            if self.right_color=='green' and self.count_green>=7:
                routine = release_red_routines[self.count_red]
                self.count_red += 1
            elif self.right_color=='blue' and self.count_blue>=7:
                routine = release_red_routines[self.count_red]
                self.count_red += 1
            if self.right_color=='green':
                routine = release_green_routines[self.count_green]
                self.count_green += 1
            elif self.right_color=='blue':
                routine = release_blue_routines[self.count_blue]
                self.count_blue += 1
            else:
                routine = release_red_routines[self.count_red]
                self.count_red += 1
        self.release_phases = [routine[0], routine[1], routine[2], routine[0]]
        return self.s_release_0

    ## DUMMY
    def dummy_1(self, vision_info, joint_state, is_moving):
        self.pick_phases = [    [-0.15, 0.055, -0.5, 1.5, -0.5, 1.43],
                                [-0.12, 0.055, -1.0, 1.0, -0.5, 1.43],
                                [-0.15, 0.055, -0.5, 1.5, -0.5, 1.43] ]
        self.ret_state.append(self.dummy_2)
        return self.s_pick_0

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
            if vision_info.left_color!='' and vision_info.right_color!='':
                self.left_color = vision_info.left_color
                self.right_color = vision_info.right_color
            rospy.loginfo('Left color: "{}"'.format(self.left_color))
            rospy.loginfo('Right color: "{}"'.format(self.right_color))
            if (self.ctp=='203' and self.left_color=='red') or (self.ctp=='303' and self.right_color=='red'):
                return self.choose_1
            if self.ctp=='303' and self.left_color=='green' and self.count_green>=7:
                return self.choose_1
            if self.ctp=='303' and self.left_color=='blue' and self.count_blue>=7:
                return self.choose_1
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
            return self.choose_2

    # release
    def s_release_0(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.release_phases[0]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_release_0
        else:
            return self.s_release_1
    def s_release_1(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.release_phases[1]
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
            self.msg.data = self.release_phases[2]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<5 or is_moving:
            return self.s_release_2
        else:
            return self.s_release_3
    def s_release_3(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.msg.data = self.release_phases[3]
            self.pub_move.publish(self.msg)
            self.timer1 = 0
        self.timer1 += 1
        if self.timer1<SRC_TIME or is_moving:
            return self.s_release_3
        else:
            return self.choose_1

    ## CONTENEDORES
    def s_close_gripper(self, vision_info, joint_state, is_moving):
        if self.state != self.state_1:
            self.pub_gripper.publish(0.25)
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

    def s_end_state(self, vision_info, joint_state, is_moving):
        return self.s_end_state
