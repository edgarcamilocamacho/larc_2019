#!/usr/bin/env python  

import numpy as np
import cv2
import math
from scipy import stats

import rospy
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

SETTINGS_RECEIVED = False

def larc_settings_callback(msg):
    global SETTINGS_RECEIVED, min_hsv_blue, max_hsv_blue, min_hsv_green, max_hsv_green, min_hsv_red, max_hsv_red, min_hsv_black, max_hsv_black
    if not SETTINGS_RECEIVED:
        print('Config received.')
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

rospy.init_node('larc_vision_node', anonymous=True)
# rate = rospy.Rate(100.0)
pub = rospy.Publisher('/larc_vision_info', LarcVisionInfo, queue_size=1)
rospy.Subscriber("/larc_settings", LarcSettings, larc_settings_callback)

cap = cv2.VideoCapture(0)

# cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
# cap.set(cv2.CAP_PROP_CONTRAST, 0)
# cap.set(cv2.CAP_PROP_SATURATION, 0)
# cap.set(cv2.CAP_PROP_HUE, 0)
# cap.set(cv2.CAP_PROP_GAIN, 0)
# cap.set(cv2.CAP_PROP_EXPOSURE, 0)
# cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
# cap.set(cv2.CAP_PROP_WHITE_BALANCE, 0)

# cap.set(cv2.CAP_PROP_BRIGHTNESS,0)
# cap.set(cv2.CAP_PROP_GAIN, 0) 
frame_counter = 0

print('Waiting for settings...')

kernel_strip = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 11, 11 ) )
kernel_cont = cv2.getStructuringElement( cv2.MORPH_ELLIPSE, ( 7, 7 ) )

while not rospy.is_shutdown():
    if SETTINGS_RECEIVED:
        black_strip_flag = False
        black_strip_x = 0
        black_strip_y = 0

        ret, frame_bgr = cap.read()
        frame_bgr[400:, :] = [255, 255, 255]
        frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        ## BLUE
        mask_blue = cv2.inRange(frame_hsv, min_hsv_blue, max_hsv_blue)
        mask_blue = cv2.dilate(mask_blue, kernel_cont, iterations = 1)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel_strip)
        mask_blue_bgr = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)

        ## GREEN
        mask_green = cv2.inRange(frame_hsv, min_hsv_green, max_hsv_green)
        mask_green = cv2.dilate(mask_green, kernel_cont, iterations = 1)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel_strip)
        mask_green_bgr = cv2.cvtColor(mask_green, cv2.COLOR_GRAY2BGR)

        ## BLUE and GREEN
        mask_cont = cv2.bitwise_or(mask_blue, mask_green)
        mask_cont_bgr = cv2.cvtColor(mask_cont, cv2.COLOR_GRAY2BGR)
        _, conts_bg, hierarchy = cv2.findContours(mask_cont, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        cv2.line(mask_cont_bgr, (0, Y_REF), ((mask_cont_bgr.shape[1]-1, Y_REF)), (0, 255, 255), 2)
        cv2.line(mask_cont_bgr, (X_REF, 0), (X_REF, (mask_cont_bgr.shape[0]-1)), (0, 255, 255), 2)

        big_conts = [None, None]
        areas = [-1, -1]
        for contour in conts_bg:
            area = cv2.moments(contour)['m00']
            if area > 15000:
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
        if len(big_conts)>0:
            cv2.circle(mask_cont_bgr, corner, 10, (255, 0, 0), -1)
            y = corner[1] - 2*Y_STEP
            while y > 0:
                row = mask_cont[y,:]
                row[:corner[0]] = 0
                points = np.append(points, [[(row!=0).argmax(axis=0),y]], axis=0)
                cv2.circle(mask_cont_bgr, (points[-1][0], points[-1][1]), 5, (255, 0, 255), -1)
                y -= Y_STEP
            slope, intercept, r_value, p_value, std_err = stats.linregress(points[:,1],points[:,0])
            # print(slope)

        # canny_green = cv2.Canny(mask_green, 50, 200, None, 3)
        # lines_green = cv2.HoughLines(canny_green, 5, 3*np.pi / 180, 200, None, 0, 0)
        # mask_green_show = np.copy(mask_green_bgr)
        # draw_lines(lines_green, mask_green_show)

        ## BLACK
        mask_black = cv2.inRange(frame_hsv, min_hsv_black, max_hsv_black)
        mask_black = cv2.bitwise_and(cv2.bitwise_not( cv2.bitwise_or(mask_blue, mask_green) ), mask_black)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_OPEN, kernel_strip)
        mask_black = cv2.morphologyEx(mask_black, cv2.MORPH_CLOSE, kernel_strip)
        mask_black_2 = np.copy(mask_black)
        width = mask_black_2.shape[1]
        mask_black_2[:, 0:2*(width/5)] = 0
        mask_black_2[:, 3*(width/5):width] = 0
        _, contours, hierarchy = cv2.findContours(mask_black_2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_area = 0
        max_area_i = -1
        for i, contour in enumerate(contours):
            if cv2.moments(contour)['m00'] > max_area:
                max_area_i = i
        mask_black_show = cv2.cvtColor(mask_black_2, cv2.COLOR_GRAY2BGR)
        if max_area_i>=0:
            black_strip_flag = True
            M = cv2.moments(contours[max_area_i])
            black_strip_x = int(M["m10"] / M["m00"])
            black_strip_y = int(M["m01"] / M["m00"])
            cv2.circle(mask_black_show, (black_strip_x, black_strip_y), 10, (0, 255, 0), -1)

        vis1 = np.concatenate((frame_bgr, mask_cont_bgr), axis=1) # vertically
        vis2 = np.concatenate((mask_cont_bgr, mask_black_show), axis=1) # vertically
        vis = np.concatenate((vis1, vis2), axis=0) # horizontally

        cv2.imshow('vis',vis)
        # cv2.imshow('mask_blue',mask_blue)
        # cv2.imshow('mask_green',mask_green)
        # cv2.imshow('mask_black',mask_black)
        pub.publish(    frame_counter, black_strip_flag, black_strip_x, black_strip_y,
                        len(big_conts), centers_x, centers_y, slope, corner[0], corner[1],
                        int(X_REF-corner[0]), int(Y_REF-corner[1]) )
        frame_counter+=1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    # rate.sleep()
    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()