#!/usr/bin/env python
import rospy
import numpy as np
import sys
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from pyexotica import KDLFrame
import tf   
    
    
def listen():
    rospy.init_node('ar_tag_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans_ar_map,rot_ar_map) = listener.lookupTransform('/ar_marker/4000','/map', rospy.Time(0)) #'/head_l_stereo_camera_frame', rospy.Time(0))
            tf_ar_map = KDLFrame(np.concatenate((trans_ar_map,rot_ar_map),axis=None))
            print(tf_ar_map.inverse().get_translation())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "no tf published"
        rate.sleep()

if __name__ == "__main__":
    print "starting"
    listen()