#!/usr/bin/env python
import numpy as np
# import pyexotica as exo
# from pyexotica.publish_trajectory import *

# import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
# import hsrb_interface
# import sys

#Create call back functions. Base feedback has nothing. Arm feedback is plentiful.
def active_cb_base():
    print('[ACTIVE_BASE] goal active')

def feedback_cb_base(feedback):
    print('[FEEDBACK] :' + str(feedback))

def done_cb_base(state, result):
    print('[DONE_BASE] the state is: '+str(state))
    print('[DONE_BASE] the result is: '+str(result))
    if state == 3:
        print('[DONE_BASE] continuing')
        # checker.base_finished = True
        print('[BASE FINISHED]')
        print('Finished')
        # rospy.signal_shutdown('i finished')
        #check_finished()
        return
    else:
        print('Issue arose, shutting down')
        # rospy.signal_shutdown('i finished')
        #rospy.signal_shutdown('finished base trajectory')

#BASE
def base_point(current_base_traj, current_base_velocity):
    p_base = trajectory_msgs.msg.JointTrajectoryPoint()
    p_base.velocities = current_base_velocity[0:3]
    p_base.positions = current_base_traj[0:3]
    p_base.time_from_start = rospy.Time(current_base_traj[3])
    return p_base

def calculate_velocity(pos_array, dt):
    pos_array = np.append(pos_array, [pos_array[-1]], axis=0)
    vel_array = np.diff(pos_array, axis = 0)/dt
    return vel_array

#Load trajectories into p_arm and p_base.
#takes in arm_list, list of base trajectories with form [base angle x3, time from start]
def load_base_goal(base_list,cli_base,dt):
    if base_list.any():
        base_vel = calculate_velocity(base_list, dt)
        base_pos = base_list
        goal_base = control_msgs.msg.FollowJointTrajectoryGoal()
        traj_base = trajectory_msgs.msg.JointTrajectory()
        traj_base.header.frame_id = "base_link"
        traj_base.joint_names = ["odom_x", "odom_y", "odom_t"]
        p_base_list = [base_point(base_pos[i,:],base_vel[i,:]) for i in range(len(base_list))]
        traj_base.points = p_base_list
        goal_base.trajectory = traj_base
        cli_base.send_goal(goal_base,done_cb=done_cb_base, active_cb=active_cb_base, feedback_cb=feedback_cb_base)
        return