#!/usr/bin/env python  
import rospy
import tf
import geometry_msgs.msg
import numpy as np

def ik1(l, dx):
    alpha = np.arctan(dx/l)
    dl = np.sqrt(dx**2+l**2)-l
    return alpha, dl

if __name__ == '__main__':
    rospy.init_node('tf_lookup_example')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/link_wrist_x', rospy.Time(0))
            # print(trans)
            rospy.loginfo("Distance in y is = {0:f}".format( abs(trans[1]) ) )
            print( ik1(0.348, 0.06) )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()