#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import sys

def calculate_diff(pos_array, dt):
    pos_array = np.append(pos_array, [pos_array[-1]], axis=0)
    vel_array = np.diff(pos_array, axis = 0)/dt
    return vel_array

if __name__ == '__main__':
    print(sys.argv)
    # ========USER INPUT========
    # default version of example_trajectory if no input
    def_val = 9
    # plot base?
    include_base = False
    # ==========================
    if len(sys.argv) >= 3:
        file = "example_traj/example_trajectory_t" + str(sys.argv[1]) + ".npy"
        if int(sys.argv[2]) >= 1:
            include_base = True
        soln = np.load(file, allow_pickle = False)
    else:
        # file = "example_trajectory_t" + str(def_val) + ".npy"
        file = "expected_velocity" + ".npy"
        soln = np.load(file, allow_pickle = False)
    print(np.shape(soln))

    print(file)
    print("showing base = "+ str(include_base))
    dt = 0.15
    if include_base:
        names = ['x','y','theta']
        # acceleration limits
        normalize = [0.05, 0.05, 0.5]
        arm_traj = soln[:,0:3]
    else:
        names = ['a_lift', 'a_flex', 'a_roll', 'w_flex', 'w_roll']
        # acceleration limits
        normalize = [0.1, 0.3, 1.0, 1.0, 1.0]
        # arm_traj = soln[:,3:8]
        arm_traj = soln[:,3:8]
    time_list = np.arange(0.0,dt*len(soln),dt)
    vel_array = calculate_diff(arm_traj, dt)
    # vel_array = soln
    print(vel_array)
    acc_array = calculate_diff(vel_array, dt)/normalize
    print(vel_array[:,1])
    print(acc_array[:,1])

    # ax0 = plt.subplot(311)#,sharex=ax2, sharey=ax2)
    # plt.title("position")
    # plt.plot(time_list,arm_traj, linewidth = 2)
    # ax0.grid(True, linestyle='-.')
    # ax0.legend(names, loc = "best")
    # ax0.set(xlabel = "time(seconds)", ylabel="pos")

    # ax1 = plt.subplot(312)#,sharex=ax2, sharey=ax2)
    # plt.title("velocity")
    # plt.plot(time_list,vel_array, linewidth = 2)
    # ax1.grid(True, linestyle='-.')
    # ax1.legend(names, loc = "best")
    # ax1.set(xlabel = "time(seconds)", ylabel="vel")

    # ax5 = plt.subplot(313)
    # plt.title("normalized acceleration")
    # plt.plot(time_list, acc_array, linewidth = 2)
    # ax5.grid(True, linestyle='-.')
    # ax5.legend(names, loc = "best")
    # ax5.set(xlabel = "time(seconds)", ylabel="acc")

    ax0 = plt.subplot(311)#,sharex=ax2, sharey=ax2)
    plt.title("velocity")
    plt.plot(time_list,vel_array, linewidth = 2)
    ax0.grid(True, linestyle='-.')
    ax0.legend(names, loc = "best")
    ax0.set(xlabel = "time(seconds)", ylabel="pos")

    ax1 = plt.subplot(312)#,sharex=ax2, sharey=ax2)
    plt.title("acceleration")
    plt.plot(time_list,acc_array*normalize, linewidth = 2)
    ax1.grid(True, linestyle='-.')
    ax1.legend(names, loc = "best")
    ax1.set(xlabel = "time(seconds)", ylabel="vel")

    ax5 = plt.subplot(313)
    plt.title("normalized acceleration")
    plt.plot(time_list, acc_array, linewidth = 2)
    ax5.grid(True, linestyle='-.')
    ax5.legend(normalize, loc = "best")
    ax5.set(xlabel = "time(seconds)", ylabel="acc")

    plt.show()
