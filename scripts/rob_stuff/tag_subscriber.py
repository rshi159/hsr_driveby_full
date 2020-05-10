#!/usr/bin/env python
'''
keeps track of pose of bottle and table relative to map frame.
'''
import rospy
import numpy as np
import sys
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from pyexotica import KDLFrame
import tf

# table_pose
# bottle_pose
tf_listener = None

def callback(data):
    global tf_listener
    rate = rospy.Rate(10.0)
    try:
        (trans_camera_map,rot_camera_map) = tf_listener.lookupTransform('/head_rgbd_sensor_rgb_frame', '/map', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    # tf from camera frame to map frame
    tf_camera_map = KDLFrame(np.concatenate((trans_camera_map,rot_camera_map),axis=None))
    # print(tf_tag0)
    # print(trans)
    for i in range(len(data.detections)):
        # print(data.detections[i].id, data.header.seq)
        if data.detections[i].id == (0,):
            # print(data.detections[i].pose.pose.pose.position)
            # position and orientation of tag
            p_0 = data.detections[i].pose.pose.pose.position
            o_0 = data.detections[i].pose.pose.pose.orientation
            # tf from camera to bottle
            tf_cam_tag_0 = KDLFrame([p_0.x,p_0.y,p_0.z,o_0.x,o_0.y,o_0.z,o_0.w])
    tf_map_tag_0 = tf_camera_map.inverse()*tf_cam_tag_0
    # print(tf_map_tag_0.get_translation())
    # print(tf_camera_map)
    # print(tf_tag_0_cam)
    # print(data.detections.pose)
    # for ar marker
    

def listener():
    global tf_listener
    rospy.init_node('tag_listener')
    tf_listener = tf.TransformListener()
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray,callback)
    print "Ready"
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("Starting")
    listener()