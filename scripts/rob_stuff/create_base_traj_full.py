#!/usr/bin/env python
from __future__ import print_function, division

from time import time, sleep
import math
import signal
import numpy as np

import rospy
import pyexotica as exo
from pyexotica.publish_trajectory import *
from pyexotica.tools import *

from pyexotica.publish_trajectory import sig_int_handler

use_screenshot = False
# ffmpeg -r 50 -f image2 -s 1920x1080 -i ./hsr_driveby_visualisation_%03d.png -vcodec libx264 -pix_fmt yuv420p ./output.mp4
screenshot = lambda *args: None
if use_screenshot:
    from jsk_rviz_plugins.srv import Screenshot, ScreenshotRequest, ScreenshotResponse
    rospy.wait_for_service('/rviz/screenshot')
    screenshot = rospy.ServiceProxy('/rviz/screenshot', Screenshot)

class DrivebyPickAndPlaceDemo(object):
    def __init__(self, debug=False):
        # Load solvers
        self.ompl_solver = exo.Setup.load_solver(
            '{hsr_driveby_full}/resources/create_base_traj.xml')
        self.ompl_problem = self.ompl_solver.get_problem()

    def solve_rrtconnect(self, start, goal):
        self.ompl_problem.goal_state = goal
        scene = self.ompl_problem.get_scene()
        # x_start[2] = -np.pi/2.
        self.ompl_problem.start_state = start
        scene.set_model_state(planner.ompl_problem.start_state)
        solution = self.ompl_solver.solve()
        return solution

# create a trajectory file and write the base trajectory to it.
def to_traj_file(soln, filename):
        import sys
        import os

        rows, cols = soln.shape
        # remove additional columns
        if cols > 4:
            soln = soln[:,cols-3:cols]
            print("trimmed the columns")
        
        traj_file = os.path.abspath(filename)
        # write to .traj file
        traj_file = open(filename, "w")
        traj_file.write("1\n")
        traj_file.write("%d %d\n" %(rows, cols))
        for i in range(rows):
            traj_file.write("%f %f %f %f\n" %(soln[i,0], soln[i,1], soln[i,2], soln[i,3]))
        traj_file.close()

if __name__ == "__main__":
    exo.Setup.init_ros()

    planner = DrivebyPickAndPlaceDemo(debug=True)
    planner2 = DrivebyPickAndPlaceDemo(debug=True)

    start = [-2.1, 1.2, -1.57]
    goal = [-0.86,0.5,-1.57]
    start2 = [-0.86,0.5,-1.57]
    goal2 = [-0.86,-1.2,-1.57]
    # create a .traj file
    
    # use rrtconnect to find a valid base trajectory that approaches the object
    solution_approach = planner.solve_rrtconnect(start, goal)
    solution_approach2 = planner2.solve_rrtconnect(start2, goal2)

    # remove redundant start point
    solution_approach2= np.delete(solution_approach2,0,axis=0)

    whole_solution = np.concatenate((solution_approach,solution_approach2))
    # print(whole_solution)
    plot(whole_solution)


    row, col = whole_solution.shape
    vel_limit = 0.5
    diff = np.diff(whole_solution, axis=0)
    print("===================")
    print(diff)
    distance = pow(diff,2)
    distance = np.sum(distance, axis=1)
    distance = np.sqrt(distance)
    print(distance)
    distance = np.insert(distance,0,0)
    # time_arr = np.arange(row) * 0.5
    dt = distance / vel_limit
    time_arr = np.cumsum(dt)
    solution = np.insert(whole_solution,0, time_arr, axis=1)
    print(solution)
    # publish_trajectory(whole_solution, 10.0, planner.ompl_solver.get_problem())
    publish_trajectory(whole_solution, time_arr[-1], planner.ompl_solver.get_problem())
    print("to traj file")

    # output base trajectory to traj file.
    to_traj_file(solution, "base_trajectory/base_trajectory_file.traj")
    print("sent to traj file. look for it now")
    # publish_trajectory(solution_approach[:,1:4], planner.t_total, planner.ompl_problem)

# create a base trajectory file for the driveby pick-and-place.
# takes in array of [start1, goal1, start2, goal2], each a base pose array with
#   form of [x,y, theta]
# outputs grasp time and duration of trajectory
def create_base_traj(goals):
    exo.Setup.init_ros()

    planner = DrivebyPickAndPlaceDemo(debug=True)
    # planner2 = DrivebyPickAndPlaceDemo(debug=True)

    start = goals[0]
    goal = goals[1]
    start2 = goals[2]
    goals2 = goals[3]
    print(start)
    print(goal)

    # create a .traj file
    
    # use rrtconnect to find a valid base trajectory that approaches the object
    solution_approach = planner.solve_rrtconnect(start, goal)
    solution_approach2 = planner.solve_rrtconnect(start2, goal2)

    row,col = np.shape(solution_approach)
    # remove redundant start point
    solution_approach2= np.delete(solution_approach2,0,axis=0)

    whole_solution = np.concatenate((solution_approach,solution_approach2))
    
    plot(whole_solution)

    # save the index of the transition between the two rrtconnect trajectories
    #   since it is returned
    row, col = whole_solution.shape

    # set the desired velocity value for the entire trajectory
    vel_limit = 0.5
    # find find the xy velocity
    diff = np.diff(whole_solution, axis=0)
    distance = pow(diff,2)
    distance = np.sum(distance, axis=1)
    distance = np.sqrt(distance)
    distance = np.insert(distance,0,0)
    # create a time array to set velocity to vel_limit
    dt = distance / vel_limit
    time_arr = np.cumsum(dt)
    # set the time in the trajectory file to the time array
    solution = np.insert(whole_solution,0, time_arr, axis=1)

    # output base trajectory to traj file.
    to_traj_file(solution, "base_trajectory/base_trajectory_file.traj")
    
    print(">>Created Base Trajectory<<")

    # return time to grasp object and duration of trajectory.
    return time_arr[row], time_arr[-1]