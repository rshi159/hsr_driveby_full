#!/usr/bin/env python
import numpy as np
import pyexotica as exo
from pyexotica.publish_trajectory import *
import tf

import rospy
import actionlib
import control_msgs.msg
import tmc_control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
from geometry_msgs.msg import PoseStamped,Pose,Quaternion,Point

import hsrb_interface
from hsrb_interface import geometry
import sys

# import roslaunch

import my_arm_client as arm
import my_base_client as base
import hsr_meeting_table_aico_whole as planner
# import my_gripper_client as gripper

def setup():
    '''Starts Simple action clients for arm, base, and gripper_traj(follow trajectory), gripper_force (apply_force) commands. Returns
    action clients for arm, base, gripper, gripper_force in that order.
    '''
    #start node
    print("node started") 
    # rospy.init_node('my_program', disable_signals=True)
    # initialize the action client
    cli_arm = actionlib.SimpleActionClient(
        '/hsrb/arm_trajectory_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    cli_base = actionlib.SimpleActionClient(
        '/hsrb/omni_base_controller/follow_joint_trajectory',
        control_msgs.msg.FollowJointTrajectoryAction)
    # initialize action client
    # ======Gripper Action Clients not Working======
    # cli_grip_follow_traj = actionlib.SimpleActionClient(
    #     '/hsrb/gripper_controller/follow_joint_trajectory',
    #     control_msgs.msg.FollowJointTrajectoryAction)
    # cli_grip_force = actionlib.SimpleActionClient(
    #     '/hsrb/gripper_controller/apply_force',
    #     tmc_control_msgs.msg.GripperApplyEffortAction)
    # =====workaround using hsrb interface
    print("waiting for server")
    #wait for the action server to establish connection
    cli_arm.wait_for_server()
    print("arm connected")
    cli_base.wait_for_server()
    print("base connected")
    # workaround
    # cli_grip_follow_traj.wait_for_server()
    # print("gripper_follow_traj connected")
    # cli_grip_force.wait_for_server()
    # print("gripper_force connected")
    #make sure the controller is running
    rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
    list_controllers = (
        rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                        controller_manager_msgs.srv.ListControllers))
    print("controller running")
    # print(list_controllers().controller)
    running_arm = False
    running_base = False
    running_gripper = False
    running_gripper_force = False
    #condense these into one while loop for asthetic
    while running_arm is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'arm_trajectory_controller' and c.state == 'running':
                running_arm = True
    while running_base is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'omni_base_controller' and c.state == 'running':
                running_base = True
    # while running_gripper is False:
    #     rospy.sleep(0.1)
    #     for c in list_controllers().controller:
    #         if c.name == 'gripper_controller' and c.state == 'running':
    #             running_gripper = True
    # while running_gripper_force is False:
    #     rospy.sleep(0.1)
    #     for c in list_controllers().controller:
    #         if c.name == 'gripper_controller' and c.state == 'running':
    #             running_gripper_force = True
    print(running_arm)
    print(running_base)
    print(running_gripper)
    print(running_gripper_force)
    print((running_base is False and running_gripper is False))
    print("gripper running")
    print("finished intializaion")

    # workaround
    # return cli_arm, cli_base, cli_grip_follow_traj, cli_grip_force
    return cli_arm, cli_base

def open_gripper(hsrb_gripper):
    # approximate radians of open gripper
    _GRASP_OPEN = 0.8
    hsrb_gripper.command(_GRASP_OPEN)

def close_gripper(hsrb_gripper):
    # Grasp force[N]
    _GRASP_FORCE=0.2
    hsrb_gripper.apply_force(_GRASP_FORCE, delicate = True)

def transform_base_traj(base_pose, tf_base):
    '''takes input of base_pose which is an array [x,y,rot] and a KDLFrame.
        returns [x,y,z,r,p,yaw]
    '''
    # frame = exo.KDLFrame(np.concatenate((base_pose[0:2],[0,0,0],base_pose[2]),axis = 0))
    frame = exo.KDLFrame(np.concatenate((base_pose[0:2],[0,0,0],[base_pose[2]])))
    print(frame)
    rel_goal = tf_base * frame
    return rel_goal.get_translation_and_rpy()

# workaround
# def close_gripper(cli_grip_force):
#     goal = tmc_control_msgs.msg.GripperApplyEffortGoal()
#     goal.effort = 1.0
#     # send message to the action server
#     cli_grip_force.send_goal(goal)
#     return

def to_file(soln, filename):
    import sys
    import os

    rows, cols = soln.shape
    traj_file = os.path.abspath(filename)
    traj_file = open(filename, "w")
    for i in range(rows):
        traj_file.write("%E %E %E\n" %(soln[i,0], soln[i,1], soln[i,2]))
    traj_file.close()

def send_trajectory(received_traj,grasp_times, dt):
    robot = hsrb_interface.Robot()
    whole_body = robot.get('whole_body')
    hsrb_gripper = robot.get('gripper')
    omni_base = robot.get('omni_base')
    try:
        open_gripper(hsrb_gripper)
        whole_body.move_to_go()
    except:
        tts.say('Fail to initialize.')
        rospy.logerr('fail to init')
        sys.exit()
    # print('going to start_location')
    # listener = tf.TransformListener()
    # listener.waitForTransform('/map','/my_start_pos', rospy.Time(0),rospy.Duration(3.0))
    # (start_pos, start_quat) = listener.lookupTransform('/map','/my_start_pos', rospy.Time(0))
    # start_state = exo.KDLFrame(start_pos + start_quat)
    # x,y,z,roll,pitch,yaw = start_state.get_translation_and_rpy()
    # print(yaw)
    # omni_base.go_abs(x,y,yaw,1000.0)
    # whole_body.move_to_go()
    whole_body.move_to_neutral()
    # exit()
    '''takes input of trajectory from hsr_meeting_table_aico, [grasp_start, grasp_duration]
        and time step of trajectory in seconds.
    '''
    #default dt = 0.1. set to >0.1 for testing / velocity limits exceeded
    dt = 0.2
    # dt = 0.10
    print("received")
    print(np.shape(received_traj))
    # midpoint = grasp_times[1]/2
    grasp_t = grasp_times# + midpoint

    # workaround
    # cli_arm, cli_base, cli_grip_follow_traj, cli_grip_force = setup()
    cli_arm, cli_base = setup()
    
    base_posestamped = rospy.wait_for_message('/global_pose', PoseStamped)

    base_pose = base_posestamped.pose

    x = base_pose.position.x
    y = base_pose.position.y
    z = base_pose.position.z
    qx= base_pose.orientation.x
    qy= base_pose.orientation.y
    qz= base_pose.orientation.z
    qw= base_pose.orientation.w
    tf_base = exo.KDLFrame([x,y,z,qx,qy,qz,qw])

    time_list = np.arange(0.0,dt*len(received_traj),dt)

    arm_list = received_traj[:,4:9]
    base_list = received_traj[:,1:4]
    arm_list = np.c_[arm_list,time_list]

    base_list= base_list - base_list[0]
    # base_list = base_list - base_list[0]
    to_file(base_list, "base_list.txt")
    # move relative to hsrb_base_link instead of map
    # base_list = np.array([transform_base_traj(base_list[i], tf_base) for i in range(len(base_list))])

    # to_file(base_list, "base_list_t.txt")
    #index of x,y,yaw
    col_index = [0,1,5]
    # base_list = base_list[:, col_index]
    base_list = np.c_[base_list, time_list]
    # #remove the zero-th time step because it has an undefined velocity
    base_list = base_list[1::]
    arm_list = arm_list[1::]

    # exit()

    print("you can run ./log_vel.py now.")
    print("press enter to continue")
    raw_input()
    print("loading arm goal")
    arm.load_arm_goal(arm_list,cli_arm,dt)
    print("loading base goal")
    base.load_base_goal(base_list,cli_base,dt)

    print("loading gripper goal")
    # gripper.load_gripper_goal(gripper_list,cli_grip_follow_traj)
    #open gripper
    # workaround
    # gripper.load_gripper_goal(np.array([[0.8,0.5]]),cli_grip_follow_traj)
    open_gripper(hsrb_gripper)
    print("sleeping")
    # gripper state = open
    gripper_state = 0
    # control gripper with this weirdness since action server for gripper doesnt work
    if len(grasp_t) > 1:
        gripper_action_times=np.diff(grasp_t)
        for i in range(len(grasp_t)):
            rospy.sleep(gripper_action_times*dt/0.1)
            if gripper_state == 0:
                close_gripper(hsrb_gripper)
            else:
                open_gripper(hsrb_gripper)
    print("closing")
    # close_gripper(cli_grip_force)
    while not rospy.is_shutdown():
        rospy.spin()
    print('All Done :)')
    print('shutting down rospy')
    exit()

if __name__ == '__main__':
    soln = np.load('whole_soln.npy', allow_pickle = False)
    # soln = planner.start_aico()
    send_trajectory(soln,[5.4, 16.8], 0.1)


