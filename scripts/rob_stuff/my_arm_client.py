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

def feedback_cb_arm(feedback):
    return
    print('[FEEDBACK] :' + str(feedback))

def done_cb_arm(state, result):
    print('[DONE_ARM] the state is: '+str(state))
    print('[DONE_ARM] the result is: '+str(result))
    if state == 3:
        print('[DONE_ARM] continuing')
        # checker.arm_finished = True
        print('[ARM FINISHED]')
        print('Finished')
        rospy.signal_shutdown('i finished')
        #check_finished()
        return
    else:
        print('Issue arose, shutting down')
        # rospy.signal_shutdown('i finished')

def active_cb_arm():
    print('[ACTIVE_ARM] arm goal active')

def arm_point(current_arm_traj, current_arm_velocity):
    #print(current_base_traj)
    p_arm = trajectory_msgs.msg.JointTrajectoryPoint()
    p_arm.velocities = current_arm_velocity[0:5]
    # p_arm.accelerations = [100,100,100,100,100]
    p_arm.positions = current_arm_traj[0:5]
    #print(p_base.positions)
    p_arm.time_from_start = rospy.Time(current_arm_traj[5])
    #print(p_base.time_from_start)
    return p_arm

def calculate_velocity(pos_array, dt):
    pos_array = np.append(pos_array, [pos_array[-1]], axis=0)
    vel_array = np.diff(pos_array, axis = 0)/dt
    return vel_array

#takes in arm_list, list of arm trajectories with form [joint angle x5, time from start]
def load_arm_goal(arm_list,cli_arm,dt):
    if arm_list.any():
        arm_vel = calculate_velocity(arm_list,dt)
        arm_pos = arm_list
        goal_arm = control_msgs.msg.FollowJointTrajectoryGoal()
        traj_arm = trajectory_msgs.msg.JointTrajectory()
        traj_arm.joint_names = ["arm_lift_joint", "arm_flex_joint",
                    "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p_arm_list = [arm_point(arm_pos[i,:], arm_vel[i,:]) for i in range(len(arm_list))]
        traj_arm.points = p_arm_list
        goal_arm.trajectory = traj_arm
        cli_arm.send_goal(goal_arm,done_cb=done_cb_arm, active_cb=active_cb_arm, feedback_cb=feedback_cb_arm)
        return