#!/usr/bin/env python
'''
takes in bottle position and table position and rotation. creates a trajectory file
for hsr_meeting_room_table_aico.
'''
import rospy
import numpy as np
from pyexotica import KDLFrame
import geometry_msgs.msg
import tf
import tf2_ros
from math import pi

def publish(object_location, name, parent):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent
    static_transformStamped.child_frame_id = name

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

rospy.init_node('start_and_end')
listener = tf.TransformListener()
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        # get tranforms of bottle and table and tag_1 (edge of table) relative to map
        (table_pos,table_quat) = listener.lookupTransform('/map','/table', rospy.Time(0))
        (bottle_pos, bottle_quat)= listener.lookupTransform('/map','/hsr_soda', rospy.Time(0))
        (table_edge_pos, table_edge_quat) = listener.lookupTransform('/map','/tag_1', rospy.Time(0))
        table = KDLFrame(table_pos + table_quat)
        bottle = KDLFrame(bottle_pos+bottle_quat)
        tag_1 = KDLFrame(table_edge_pos + table_edge_quat)
        # table relative to bottle. Can find how to rotate bottle tf to match table
        # how to rotate bottle to get to table tf
        bottle_table = bottle.inverse() * table
        # rotate the bottle tf so that it matches the orientation of the table
        bottle_with_table_orientation = KDLFrame([0.0,0.0,0.0]+bottle_table.get_quaternion().tolist())
        # [DEBUG]: 
        publish(bottle_with_table_orientation, 'bottle_with_table_orientation', 'hsr_soda')
        
        # [DEBUG]: y offest of bottle for aico. print((bottle_with_table_orientation.inverse() * bottle_table).get_translation())
        publish(bottle_with_table_orientation.inverse() * bottle_table, 'bottle_to_table','bottle_with_table_orientation') 
        # tag_1 (table edge) relative to bottle
        bottle_tag1 = bottle.inverse() * tag_1
        # z is the perpendicular distance from the bottle to the table, since the orientations are matched
        # #table_T_bottle * bottle-T-bottle_rotated? #used wrong table tf. center instead of tag
        # bottle_with_t_ori-T-bottle * bottle-T-table_edge
        x,y,z = (bottle_with_table_orientation.inverse() * bottle_tag1).get_translation()
        # [DEBUG]: print(z)
        # publish (bottle_with_table_orientation.inverse() * bottle_tag1, 'zzzzzzzzzz', 'bottle_with_table_orientation')
        
        # hard coded offset from bottle location. Include z offset to make trajectory path
        # aligned with (x distance to bottle) and (y distance to table)
        # make it so that grasping time in aico solver works and we can use the same input trajectory.
        # include 3.5 cm offset cuz it works better
        start_from_bottle = KDLFrame([0.9,0.0,0.4+z-0.035,0.0,0.0,0.0,1.0])
        # get the starting x and y position frame
        start_position = bottle_with_table_orientation * start_from_bottle
        start_orientation = KDLFrame([0.0,0.0,0.0]+tf.transformations.quaternion_from_euler(pi/2, 0, -pi).tolist())
        start_pose_bottle = start_position * start_orientation
        # map-T-start_pose = map-T-bottle * bottle-T-start
        start_pose_map = bottle * start_pose_bottle
        start_r, start_p,stary_y = start_pose_map.get_rpy()
        # only accept possible positions if pitch and roll fall beneath arbitrary threshold.
        # if abs(start_r) < 0.01 and abs(start_p) < 0.01:
        if abs(start_r) < 0.05 and abs(start_p) < 0.05:
            # print(start_pose_map.get_rpy())
            publish(start_pose_map, 'my_start_pos', 'map')
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    rate.sleep()