#!/bin/bash

ROBOT_IP="192.168.20.77"
ROBOT_USER="larc"
ROBOT_PASS="123456789"
WORKSPACE="/home/${ROBOT_USER}/ros/larc_ws"
TO_COPY="dynamixel_controllers larc_settings larc_vision larc_control larc_rviz larc_robot_description larc_motors_control startUSB.sh"
ROBOT_ROS="kinetic"

PACKAGE=${PWD##*/}
sshpass -p ${ROBOT_PASS} ssh ${ROBOT_USER}@${ROBOT_IP} "rm -rf ${WORKSPACE}/src/${PACKAGE}"
sshpass -p ${ROBOT_PASS} ssh ${ROBOT_USER}@${ROBOT_IP} "mkdir -p ${WORKSPACE}/src/${PACKAGE}"
sshpass -p ${ROBOT_PASS} scp -r ${TO_COPY} ${ROBOT_USER}@${ROBOT_IP}:${WORKSPACE}/src/${PACKAGE}
# sshpass -p ${ROBOT_PASS} ssh ${ROBOT_USER}@${ROBOT_IP} "cd ${WORKSPACE} && /opt/ros/${ROBOT_ROS}/bin/catkin_make"

