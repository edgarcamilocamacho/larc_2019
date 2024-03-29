#!/usr/bin/env python  

import numpy as np
import cv2
import math
from scipy import stats

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from larc_vision.msg import LarcVisionInfo
from larc_settings.msg import LarcSettings

min_hsv_blue = np.array([90,30,100])
max_hsv_blue = np.array([135,255,255])

min_hsv_green = np.array([45,30,100])
max_hsv_green = np.array([75,255,255])

min_hsv_red = np.array([0,0,0])
max_hsv_red = np.array([180,255,255])

min_hsv_black = np.array([0,0,0])
max_hsv_black = np.array([180,255,255])

Y_STEP = 10

X_REF = 291
Y_REF = 244
S_REF = 0.0

SETTINGS_RECEIVED = False

def larc_settings_callback(msg):
    global SETTINGS_RECEIVED, min_hsv_blue, max_hsv_blue, min_hsv_green, max_hsv_green, min_hsv_red, max_hsv_red, min_hsv_black, max_hsv_black
    global X_REF, Y_REF, S_REF
    if not SETTINGS_RECEIVED:
        rospy.loginfo('Config received.')
    min_hsv_blue[0] = int(msg.min_hsv_blue[0]/2)
    min_hsv_blue[1] = int( 255.0*(msg.min_hsv_blue[1]/100.0) )
    min_hsv_blue[2] = int( 255.0*(msg.min_hsv_blue[2]/100.0) )
    min_hsv_green[0] = int(msg.min_hsv_green[0]/2)
    min_hsv_green[1] = int( 255.0*(msg.min_hsv_green[1]/100.0) )
    min_hsv_green[2] = int( 255.0*(msg.min_hsv_green[2]/100.0) )
    min_hsv_red[0] = int(msg.min_hsv_red[0]/2)
    min_hsv_red[1] = int( 255.0*(msg.min_hsv_red[1]/100.0) )
    min_hsv_red[2] = int( 255.0*(msg.min_hsv_red[2]/100.0) )
    min_hsv_black[0] = int(msg.min_hsv_black[0]/2)
    min_hsv_black[1] = int( 255.0*(msg.min_hsv_black[1]/100.0) )
    min_hsv_black[2] = int( 255.0*(msg.min_hsv_black[2]/100.0) )

    max_hsv_blue[0] = int(msg.max_hsv_blue[0]/2)
    max_hsv_blue[1] = int( 255.0*(msg.max_hsv_blue[1]/100.0) )
    max_hsv_blue[2] = int( 255.0*(msg.max_hsv_blue[2]/100.0) )
    max_hsv_green[0] = int(msg.max_hsv_green[0]/2)
    max_hsv_green[1] = int( 255.0*(msg.max_hsv_green[1]/100.0) )
    max_hsv_green[2] = int( 255.0*(msg.max_hsv_green[2]/100.0) )
    max_hsv_red[0] = int(msg.max_hsv_red[0]/2)
    max_hsv_red[1] = int( 255.0*(msg.max_hsv_red[1]/100.0) )
    max_hsv_red[2] = int( 255.0*(msg.max_hsv_red[2]/100.0) )
    max_hsv_black[0] = int(msg.max_hsv_black[0]/2)
    max_hsv_black[1] = int( 255.0*(msg.max_hsv_black[1]/100.0) )
    max_hsv_black[2] = int( 255.0*(msg.max_hsv_black[2]/100.0) )

    X_REF = msg.ref_x
    Y_REF = msg.ref_y
    S_REF = msg.ref_s

    SETTINGS_RECEIVED = True

def draw_lines(lines, img):
    if lines is not None:
        for i in range(0, len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
            pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
            cv2.line(img, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

rospy.init_node('larc_vision', anonymous=False)
# rate = rospy.Rate(100.0)
pub = rospy.Publisher('/larc_vision_info', LarcVisionInfo, queue_size=1)

frame_bgr_pub = rospy.Publisher("/larc/image/frame_bgr",Image, queue_size=1)
mask_red_pub = rospy.Publisher("/larc/image/mask_red",Image, queue_size=1)
mask_blue_pub = rospy.Publisher("/larc/image/mask_blue",Image, queue_size=1)
mask_green_pub = rospy.Publisher("/larc/image/mask_green",Image, queue_size=1)
mask_cont_pub = rospy.Publisher("/larc/image/mask_cont",Image, queue_size=1)
mask_cont_info_pub = rospy.Publisher("/larc/image/mask_cont_info",Image, queue_size=1)
mask_black_pub = rospy.Publisher("/larc/image/mask_black",Image, queue_size=1)
mask_black_info_pub = rospy.Publisher("/larc/image/mask_black_info",Image, queue_size=1)

rospy.Subscriber("/larc_settings", LarcSettings, larc_settings_callback)

cv_bridge = CvBridge()

cap = cv2.VideoCapture(0)

frame_counter = 0

rospy.loginfo('Waiting for settings...')

kernel_strip = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 11, 11 ) )
kernel_cont = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 7, 7 ) )

MIN_Y_WHITE = 60

def get_conts(mask):
    mask = cv2.dilate(mask, kernel_cont, iterations = 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_strip)
    _, conts, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    big_conts = [None, None]
    areas = [-1, -1]
    for contour in conts:
        area = cv2.moments(contour)['m00']
        if area > 12000:
            if area > areas[0]:
                big_conts[1] = np.copy(big_conts[0])
                areas[1] = areas[0]
                big_conts[0] = np.copy(contour)
                areas[0] = area
            elif area > areas[1]:
                big_conts[1] = np.copy(contour)
                areas[1] = area
    if areas[1]==-1:
        del areas[1]
        del big_conts[1]
    if areas[0]==-1:
        del areas[0]
        del big_conts[0]
    mask_2 = np.zeros_like(mask)
    cv2.drawContours(mask_2, big_conts, -1, (255), -1)
    return big_conts, mask_2


while not rospy.is_shutdown():
    if SETTINGS_RECEIVED:
        ret, frame_bgr = cap.read()
        # frame_bgr[:MIN_Y_WHITE, :] = [255, 255, 255]
        frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        ## RED
        mask_red_1 = cv2.inRange(frame_hsv, np.array([0             , min_hsv_red[1], min_hsv_red[2]]),
                                            np.array([min_hsv_red[0], max_hsv_red[1], max_hsv_red[2]]) )
        mask_red_2 = cv2.inRange(frame_hsv, np.array([max_hsv_red[0], min_hsv_red[1], min_hsv_red[2]]),
                                            np.array([255           , max_hsv_red[1], max_hsv_red[2]]) )
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        conts_r, mask_red_2 = get_conts(mask_red)
        mask_red_bgr = cv2.cvtColor(mask_red_2, cv2.COLOR_GRAY2BGR)

        ## BLUE
        mask_blue = cv2.inRange(frame_hsv, min_hsv_blue, max_hsv_blue)
        conts_b, mask_blue_2 = get_conts(mask_blue)
        mask_blue_bgr = cv2.cvtColor(mask_blue_2, cv2.COLOR_GRAY2BGR)

        ## GREEN
        mask_green = cv2.inRange(frame_hsv, min_hsv_green, max_hsv_green)
        conts_g, mask_green_2 = get_conts(mask_green)
        mask_green_bgr = cv2.cvtColor(mask_green_2, cv2.COLOR_GRAY2BGR)

        ## MIX
        mask_cont = cv2.bitwise_or(mask_blue_2, mask_green_2)
        mask_cont = cv2.bitwise_or(mask_cont, mask_red_2)
        mask_cont_bgr = cv2.cvtColor(mask_cont, cv2.COLOR_GRAY2BGR)
        mask_cont_bgr_2 = np.copy(mask_cont_bgr)
        big_conts = conts_r + conts_g + conts_b

        cv2.line(mask_cont_bgr, (0, Y_REF), ((mask_cont_bgr.shape[1]-1, Y_REF)), (0, 255, 255), 2)
        cv2.line(mask_cont_bgr, (X_REF, 0), (X_REF, (mask_cont_bgr.shape[0]-1)), (0, 255, 255), 2)
        cv2.line(mask_cont_bgr, (int(X_REF-S_REF*Y_REF), 0), (X_REF, Y_REF), (0, 0, 255), 3)

        contours_poly = [None]*len(big_conts)
        boundRect = [None]*len(big_conts)
        centers_x = [None]*len(big_conts)
        centers_y = [None]*len(big_conts)
        corner = (10000, 10000)
        for i, c in enumerate(big_conts):
            contours_poly[i] = cv2.approxPolyDP(c, 4, True)
            boundRect[i] = cv2.boundingRect(contours_poly[i])
            M = cv2.moments(c)
            centers_x[i] = int(M["m10"] / M["m00"])
            centers_y[i] = int(M["m01"] / M["m00"])
            if int(boundRect[i][0])<corner[0]:
                corner = (int(boundRect[i][0]), int(boundRect[i][1]+boundRect[i][3]))


        for i in range(len(big_conts)):
            color = (255, 255, 0)
            cv2.rectangle(mask_cont_bgr, (int(boundRect[i][0]), int(boundRect[i][1])), \
            (int(boundRect[i][0]+boundRect[i][2]), int(boundRect[i][1]+boundRect[i][3])), color, 2)
            cv2.circle(mask_cont_bgr, (centers_x[i], centers_y[i]), 10, (0, 0, 255), -1)
        
        slope = float('nan')
        points = np.empty((0,2), dtype=int)
        if len(big_conts)>=2:
            y = corner[1] - 2*Y_STEP
            while y > MIN_Y_WHITE:
                row = mask_cont[y,:]
                row[:corner[0]] = 0
                points = np.append(points, [[(row!=0).argmax(axis=0),y]], axis=0)
                cv2.circle(mask_cont_bgr, (points[-1][0], points[-1][1]), 5, (255, 0, 255), -1)
                y -= Y_STEP
            try:
                slope, intercept, r_value, p_value, std_err = stats.linregress(points[:,1],points[:,0])
                corner = (points[0][0], corner[1])
                cv2.circle(mask_cont_bgr, corner, 10, (255, 0, 0), -1)
            except:
                pass
            # print(slope)
            

        ## Check container color
        
        if len(big_conts)==2:

            if centers_x[0]>centers_x[1]:
                idx_left = 1
                idx_right = 0 
            else:
                idx_left = 0
                idx_right = 1

            # left 
            mask_left = np.zeros_like(mask_blue)
            cv2.drawContours(mask_left, [big_conts[idx_left]], -1, (255), -1)
            pix_blue = np.sum(np.sum(cv2.bitwise_and(mask_left, mask_blue)))
            pix_green = np.sum(np.sum(cv2.bitwise_and(mask_left, mask_green)))
            pix_red = np.sum(np.sum(cv2.bitwise_and(mask_left, mask_red)))
            if pix_blue>pix_green and pix_blue>pix_red:
                left_color = 'blue'
            elif pix_green>pix_blue and pix_green>pix_red:
                left_color = 'green'
            else:
                left_color = 'red'
            
            # right 
            mask_right = np.zeros_like(mask_blue)
            cv2.drawContours(mask_right, [big_conts[idx_right]], -1, (255), -1)
            pix_blue = np.sum(np.sum(cv2.bitwise_and(mask_right, mask_blue)))
            pix_green = np.sum(np.sum(cv2.bitwise_and(mask_right, mask_green)))
            pix_red = np.sum(np.sum(cv2.bitwise_and(mask_right, mask_red)))
            if pix_blue>pix_green and pix_blue>pix_red:
                right_color = 'blue'
            elif pix_green>pix_blue and pix_green>pix_red:
                right_color = 'green'
            else:
                right_color = 'red'
        else:
            left_color = ''
            right_color = ''

        ## BLACK
        mask_black = cv2.inRange(frame_hsv, min_hsv_black, max_hsv_black)
        mask_black = cv2.bitwise_and(cv2.bitwise_not( cv2.bitwise_or(mask_blue, mask_green) ), mask_black)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel_strip)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel_strip)
        mask_black_2 = np.copy(mask_black)
        width = mask_black_2.shape[1]
        mask_black_2[:, 0:2*(width/5)] = 0
        mask_black_2[:, 3*(width/5):width] = 0
        mask_black_show = cv2.cvtColor(mask_black_2, cv2.COLOR_GRAY2BGR)
        _, contours, hierarchy = cv2.findContours(mask_black_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        black_strip_x = 0
        black_strip_y = 0
        black_strip_flag = False
        for i, contour in enumerate(contours):
            area = cv2.moments(contour)['m00']
            if area > 1500:
                black_strip_flag = True
                M = cv2.moments(contour)
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                if y>black_strip_y:
                    black_strip_x = x
                    black_strip_y = y
        
        if black_strip_flag:
            cv2.circle(mask_black_show, (black_strip_x, black_strip_y), 10, (0, 255, 0), -1)

        # vis1 = np.concatenate((frame_bgr, mask_red_bgr), axis=1) # vertically
        # vis2 = np.concatenate((mask_green_bgr, mask_blue_bgr), axis=1) # vertically
        # vis = np.concatenate((vis1, vis2), axis=0) # horizontally

        # vis1 = np.concatenate((frame_bgr, mask_cont_bgr_2), axis=1) # vertically
        # vis2 = np.concatenate((mask_cont_bgr, mask_black_show), axis=1) # vertically
        # vis = np.concatenate((vis1, vis2), axis=0) # horizontally

        # cv2.imshow('vis',vis)

        frame_bgr_pub.publish(cv_bridge.cv2_to_imgmsg(frame_bgr, "bgr8"))
        mask_red_pub.publish(cv_bridge.cv2_to_imgmsg(mask_red_bgr, "bgr8"))
        mask_blue_pub.publish(cv_bridge.cv2_to_imgmsg(mask_blue_bgr, "bgr8"))
        mask_green_pub.publish(cv_bridge.cv2_to_imgmsg(mask_green_bgr, "bgr8"))
        mask_cont_pub.publish(cv_bridge.cv2_to_imgmsg(mask_cont_bgr_2, "bgr8"))
        mask_cont_info_pub.publish(cv_bridge.cv2_to_imgmsg(mask_cont_bgr, "bgr8"))
        # mask_black_pub.publish(cv_bridge.cv2_to_imgmsg(mask_black_2, "bgr8"))
        mask_black_info_pub.publish(cv_bridge.cv2_to_imgmsg(mask_black_show, "bgr8"))
        
        pub.publish(    frame_counter, black_strip_flag, black_strip_x, black_strip_y,
                        len(big_conts), centers_x, centers_y, slope, corner[0], corner[1],
                        S_REF-slope, int(X_REF-corner[0]), int(Y_REF-corner[1]),
                        left_color, right_color )
        frame_counter+=1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # rate.sleep()
    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()