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

# def active_cb_gripper():
#     print('[ACTIVE_GRIPPER] grip goal active')

# def feedback_cb_gripper(feedback):
#     print('[GRIPPER FEEDBACK] :' + str(feedback))
#     return

# def done_cb_gripper(state,result):
#     print('[DONE_GRIPPER] the state is: '+str(state))
#     print('[DONE_GRIPPER] the result is: '+str(result))
#     if state == 3:
#         rospy.sleep(3)
#         print('[DONE_GRIPPER] continuing')
#         # checker.base_finished = True
#         print('[GRIPPER FINISHED]')
#         print('Finished')
#         # rospy.signal_shutdown('i finished my stuff')
#         #check_finished()
#         return
#     else:
#         print('Issue arose with gripper, shutting down')
#         # rospy.signal_shutdown('i finished poorly')
#         #rospy.signal_shutdown('finished base trajectory')

# def grip_point(current_grip_traj):
#     # print(current_grip_traj)
#     p_grip = trajectory_msgs.msg.JointTrajectoryPoint()
#     p_grip.positions = [current_grip_traj[0]]
#     p_grip.velocities = [0]
#     p_grip.effort = [0.5]
#     p_grip.time_from_start = rospy.Time(current_grip_traj[1])
#     # print(p_grip.time_from_start)
#     # print(p_grip)
#     return p_grip
# def load_gripper_goal(gripper_list, cli_gripper):
#     if gripper_list.any():
#         # fill ROS message
#         goal_grip = control_msgs.msg.FollowJointTrajectoryGoal()
#         traj_grip = trajectory_msgs.msg.JointTrajectory()
#         traj_grip.joint_names = ["hand_motor_joint"]
#         traj_grip.header.stamp = rospy.Time.now()
#         traj_grip_list = [grip_point(gripper_list[i]) for i in range(len(gripper_list))]
#         # print(traj_grip_list)
#         # print(type(traj_grip_list))
#         traj_grip.points = traj_grip_list
#         goal_grip.trajectory = traj_grip
#         # send message to the action server
#         cli_gripper.send_goal(goal_grip,done_cb = done_cb_gripper,feedback_cb=feedback_cb_gripper,active_cb=active_cb_gripper)
#         return

