#!/usr/bin/env python
'''
reads tag data from apriltag_ros node /tag_detections. reads relative positions of
table and hsr_soda from YAML file and publishes location of objects.
'''
import sys
import yaml
import rospy
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from apriltag_ros.msg import AprilTagDetection
from pyexotica import KDLFrame
import geometry_msgs.msg
import tf
import tf2_ros
from math import pi

stream = file('/home/rs36/catkin_ws/src/hsr_driveby_full/object/marker.yaml', 'r')
# relative locations of table and bottle with respect to tags.
rel = yaml.load(stream) 
# print(rel[0].get('marker'))
# print(type(rel[0].get('marker')))
# print(type(rel))#.get('object_id'))
tf_listener = None

debug = False


def callback(data):
    global rel
    global tf_listener
    global debug

    rate = rospy.Rate(10.0)
    try:
        (trans_camera_map,rot_camera_map) = tf_listener.lookupTransform('/head_rgbd_sensor_rgb_frame', '/map', rospy.Time(0))
        # tf from camera frame to map frame
        tf_camera_map = KDLFrame(np.concatenate((trans_camera_map,rot_camera_map),axis=None))
        # print(tf_tag0)
        # print(trans)
        for i in range(len(data.detections)):
            # print(data.detections[i].id, data.header.seq)
            # if data.detections[i].id == (0,):
            marker_id = data.detections[i].id
            # rel_id = next(x for x in rel if x.get('object_id') == marker_id)
            try:
                if debug:
                    print(marker_id[0])
                # rel_id = [i for i,val in rel if val.get('object_id')==marker_id[0]]
                rel_id = next(i for i,x in enumerate(rel) if x.get('object_id') == marker_id[0])
                if debug:
                    print(rel_id)
            except:
                print("problem in rel_id")
                break
            if debug:
                print(rel[rel_id])
            rel_p = rel[rel_id].get('marker')[0].get('translation')
            rel_o = rel[rel_id].get('marker')[0].get('rotation')
            tf_relative = KDLFrame(rel_p + rel_o)
            # print(data.detections[i].pose.pose.pose.position)
            # position and orientation of tag
            p_0 = data.detections[i].pose.pose.pose.position
            o_0 = data.detections[i].pose.pose.pose.orientation
            # tf from camera to object
            # orientation of apriltags is not great. set to (1,0,0,0) frame orientation
            # tf_cam_tag_0 = KDLFrame([p_0.x,p_0.y,p_0.z,o_0.x,o_0.y,o_0.z,o_0.w])
            tf_cam_tag_0 = KDLFrame([p_0.x,p_0.y,p_0.z,o_0.x,o_0.y,o_0.z,o_0.w])
            # x,y,z,tag_r, tag_p, tag_y = tf_cam_tag_0.get_translation_and_rpy()
            # print(tag_r)
            # print(tag_p)
            # print(tag_y)
            # # tag_r = -pi
            # tag_p = 0.0
            # tag_y = 0.0
            # tf_cam_tag_0 = KDLFrame([x,y,z]+tf.transformations.quaternion_from_euler(tag_r,tag_p,tag_y).tolist())
            tf_map_tag_0 = tf_camera_map.inverse()*tf_cam_tag_0
            tf_map_tag_0.get_translation_and_rpy()
            object_location = tf_map_tag_0 * tf_relative
            x,y,z,tag_r, tag_p, tag_y = object_location.get_translation_and_rpy()
            if debug:
                print(tag_r)
                print(tag_p)
                print(tag_y)
            # object_location = tf_map_tag_0
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transformStamped = geometry_msgs.msg.TransformStamped()

            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "map"
            static_transformStamped.child_frame_id = rel[rel_id].get('name')


            static_transformStamped.transform.translation.x = object_location.get_translation()
            static_transformStamped.transform.rotation.w = object_location.get_quaternion()


            static_transformStamped.transform.translation.x = float(object_location.get_translation()[0])
            static_transformStamped.transform.translation.y = float(object_location.get_translation()[1])
            static_transformStamped.transform.translation.z = float(object_location.get_translation()[2])

            static_transformStamped.transform.rotation.x = object_location.get_quaternion()[0]
            static_transformStamped.transform.rotation.y = object_location.get_quaternion()[1]
            static_transformStamped.transform.rotation.z = object_location.get_quaternion()[2]
            static_transformStamped.transform.rotation.w = object_location.get_quaternion()[3]

            broadcaster.sendTransform(static_transformStamped)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("nothing detected")
        pass


def listen():
    global tf_listener
    print "Ready"
    rospy.init_node('object_postions')
    tf_listener = tf.TransformListener()
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray,callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    # listener = tf.TransformListener()
    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         (trans_ar_map,rot_ar_map) = listener.lookupTransform('/ar_marker/4000','/map', rospy.Time(0)) #'/head_l_stereo_camera_frame', rospy.Time(0))
    #         tf_ar_map = KDLFrame(np.concatenate((trans_ar_map,rot_ar_map),axis=None))
    #         print(tf_ar_map.inverse().get_translation())
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         print "no tf published"
    #     rate.sleep()

if __name__ == "__main__":
    print "starting"
    listen()
# print data.get('marker')[0].get('marker_id')
# print(type(data.get('marker')[0].get('marker_id')))

'''
[{'marker': [{'translation': [0.0, 0.07, -0.35005], 'marker_id': 1, 
'rotation': [0.0, 0.0, 0.0, 1], 'size': 0.14}], 
'name': 'table', 'object_id': 1}, 

{'marker': [{'translation': [0.0, 0.0, -0.0351], 'marker_id': 4000, 
'rotation': [0.0, 0.0, 0.0, 1], 'size': 0.1}], 
'name': 'hsr_soda', 'object_id': 0}]
'''
