#!/usr/bin/env python
import rospy
# import message_filters
import numpy as np
from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryActionFeedback
from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt

x2 = np.array([])
x3 = np.array([])
joint_traj = np.array([[0,0,0,0,0]])
joint_traj_2 = np.array([[0,0,0,0,0]])
cb1 = False
names = np.array([])
names_2 = np.array([])

# ==========USER INPUTS===========
# number of datapoints to read in. 100 * dt * 10
# steps = 150
steps = 100
# plot position or velocity
plot_positions = False
# normalize
normalize = True
# plot base
# plot_base = False
# ================================

normalize_array = [0.1, 0.3, 1.0, 1.0, 1.0]
def callback_1(data):
    global cb1
    global steps
    if not cb1:
        cb1 = True
        print("STARTING")


def callback_2(data):
    global x2
    global cb1
    global joint_traj_2
    global steps
    global names_2

    if len(joint_traj_2) > steps*5:
        rospy.signal_shutdown("Finished")
    if cb1:
        if len(joint_traj) > steps+5:
            rospy.signal_shutdown("Finished")
        # print("=====cb 3=====")
        # print(data.position[14:19])
        time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        # print("===="+str(time))
        x2 = np.append(x2, time)
        # joint_traj_2 = np.append(joint_traj_2, [data.velocity[np.array([1,0,2,11,12])]], axis = 0)
        # names_2 = data.name[np.array([1,0,2,11,12])]
        idx = [1,0,2,11,12]
        joint_traj_2 = np.append(joint_traj_2, [np.array(data.velocity)[idx]], axis = 0)
        names_2 = np.array(data.name)[idx]

def callback_3(data):
    global x3
    global joint_traj
    global cb1
    global steps
    global names
    if cb1:
        if len(joint_traj) > steps+5:
            rospy.signal_shutdown("Finished")
        # print("=====cb 3=====")
        # print(data.position[14:19])
        time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        # print("===="+str(time))
        x3 = np.append(x3, time)
        if plot_positions:
            joint_traj = np.append(joint_traj, [data.position[14:19]], axis = 0)
        else:
            joint_traj = np.append(joint_traj, [data.velocity[14:19]], axis = 0)
        names = data.name[14:19]
    # print("=="+str(len(joint_traj)))
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener')

    # # 10 hz
    # # base_fb = message_filters.Subscriber('/hsrb/omni_base_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback)
    # arm_fb = message_filters.Subscriber('/hsrb/arm_trajectory_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback)
    # # 10 hz
    # joint_st = message_filters.Subscriber("/hsrb/robot_state/joint_states", JointState)

    # ts = message_filters.ApproximateTimeSynchronizer([arm_fb, joint_st], queue_size=10, slop=0.5)
    # ts.registerCallback(callback)

    # =========================================================

    # rospy.Subscriber('/hsrb/omni_base_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, callback_1)
    # rospy.Subscriber('/hsrb/arm_trajectory_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback,callback_2)
    rospy.Subscriber('/hsrb/arm_trajectory_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback,callback_1)

    rospy.Subscriber('/hsrb/joint_states', JointState,callback_2)
    # 10 hz
    rospy.Subscriber("/hsrb/robot_state/joint_states", JointState,callback_3)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    print("WAITNG")
    print("run:")
    print("    roscd /hsr_driveby_demo/scripts/robstuff")
    print('    ./hsrb_exotica_python_script.py')
    listener()
    print("FINISHED LISTENING, PLOTTING")
    cb2_len = len(x2)
    cb3_len = len(x3)
    # print(arm_traj_actual)
    # print(np.delete(arm_traj_actual,0,0))
    joint_traj = np.delete(joint_traj,0,0)[:steps-1]
    x3 = x3[:steps-1]
    x3 = x3 - x3[0]
    x2 = x2 - x2[0]
    joint_traj_2 = np.delete(joint_traj_2,0,0)
    # print(x2[:steps])
    # print(arm_traj_actual)
    print(len(x2))
    print(len(x3))
    print(len(joint_traj))
    print(len(joint_traj_2))

    print(names_2)
    ax1 = plt.subplot(231)#,sharex=ax2, sharey=ax2)
    plt.title("/hsrb/joint_states")
    print(np.shape(x2))
    print(np.shape(joint_traj_2))
    plt.plot(x2, joint_traj_2, linewidth = 2)
    ax1.set_xlim([0, x2[-11]])
    ax1.grid(True, linestyle='-.')
    ax1.legend(names_2, loc = "best")
    # ax1.set_ylim((-1, 1))


    ax2 = plt.subplot(233)#,sharex=ax2, sharey=ax2)
    plt.title("/hsrb/joint_states diffed/dt (50hz) Normalized")
    diffed = np.diff(joint_traj_2[:-11],axis=0)
    acc = diffed.transpose()/np.diff(x2[:-11])
    if normalize:
        plt.plot(x2[1:-11], acc.transpose()/normalize_array, linewidth = 2)
    else:
        plt.plot(x2[1:-11], acc.transpose(), linewidth = 2)
    # plt.plot(x2[1:-11], diffed, linewidth = 2)
    ax2.set_xlim([0, x2[-11]])
    ax2.grid(True, linestyle='-.')
    if normalize_array:
        ax2.legend(normalize_array, loc="best")
    else:
        ax2.legend(names, loc = "best")


    # ax7 = plt.subplot(232)#,sharex=ax2, sharey=ax2)
    # plt.title("/hsrb/joint_states diffed")
    # if normalize:
    #     plt.plot(x2[1:-11], diffed, linewidth = 2)
    # else:
    #     plt.plot(x2[1:-11], diffed, linewidth = 2)
    # ax7.set_xlim([0, x2[-11]])
    # ax7.grid(True, linestyle='-.')
    # if normalize_array:
    #     ax7.legend(normalize_array, loc="best")
    # else:
    #     ax7.legend(names, loc = "best")
    # ax7.set_xlim([0, x2[-11]])
    # ax7.grid(True, linestyle='-.')
    # if normalize_array:
    #     ax7.legend(names, loc="best")
    # else:
    #     ax7.legend(names, loc = "best")
    ax7 = plt.subplot(232)#,sharex=ax2, sharey=ax2)
    plt.title("/hsrb/joint_states diffed/dt")
    if normalize:
        plt.plot(x2[1:-11], acc.transpose(), linewidth = 2)
    else:
        plt.plot(x2[1:-11], acc.transpose(), linewidth = 2)
    ax7.set_xlim([0, x2[-11]])
    ax7.grid(True, linestyle='-.')
    if normalize_array:
        ax7.legend(normalize_array, loc="best")
    else:
        ax7.legend(names, loc = "best")
    ax7.set_xlim([0, x2[-11]])
    ax7.grid(True, linestyle='-.')
    if normalize_array:
        ax7.legend(names, loc="best")
    else:
        ax7.legend(names, loc = "best")


    ax5 = plt.subplot(234)
    plt.title("/hsrb/robot_states/joint_state")
    plt.plot(x3[:steps-1], joint_traj[:steps-1], linewidth = 2)
    ax5.grid(True, linestyle='-.')
    ax5.legend(names, loc = "best")
    ax5.set_xlim([0, x3[-1]])



    ax3 = plt.subplot(236)
    plt.title("/hsrb/robot_states/joint_state diffed/dt  (10hz) Normalized")
    diffed_2 = np.diff(joint_traj[:steps-1],axis=0)
    acc_2 = diffed_2.transpose()/np.diff(x3[:steps-1])
    print(np.shape(acc_2.transpose()))
    print(np.shape(x3[1:steps-1]))
    if normalize:
        plt.plot(x3[1:steps-1], acc_2.transpose()/normalize_array, linewidth = 2)
    else:
        plt.plot(x3[1:steps-1], acc_2.transpose(), linewidth = 2)
    # plt.plot(x3[1:steps-1], diffed_2, linewidth = 2)
    ax3.grid(True, linestyle='-.')
    if normalize_array:
        ax3.legend(normalize_array, loc="best")
    else:
        ax3.legend(names, loc = "best")
    ax3.set_xlim([0, x3[-1]])

    # ax9 = plt.subplot(235)
    # plt.title("/hsrb/robot_states/joint_state diffed")
    # if normalize:
    #     plt.plot(x3[1:steps-1], diffed_2, linewidth = 2)
    # else:
    #     plt.plot(x3[1:steps-1], diffed_2, linewidth = 2)
    # # plt.plot(x3[1:steps-1], diffed_2, linewidth = 2)
    # ax9.grid(True, linestyle='-.')
    # if normalize_array:
    #     ax9.legend(names, loc="best")
    # else:
    #     ax9.legend(names, loc = "best")
    # ax9.set_xlim([0, x3[-1]])
    ax9 = plt.subplot(235)
    plt.title("/hsrb/robot_states/joint_state diffed/dt")
    if normalize:
        plt.plot(x3[1:steps-1], acc_2.transpose(), linewidth = 2)
    else:
        plt.plot(x3[1:steps-1], acc_2.transpose(), linewidth = 2)
    # plt.plot(x3[1:steps-1], diffed_2, linewidth = 2)
    ax9.grid(True, linestyle='-.')
    if normalize_array:
        ax9.legend(names, loc="best")
    else:
        ax9.legend(names, loc = "best")
    ax9.set_xlim([0, x3[-1]])

    # print(diffed[:10])
    # print(acc.transpose()[:10])
    # print(np.diff(x2[:-11])[:10])
    # print(diffed_2[:10])
    # print(acc_2.transpose()[:10])
    # print(np.diff(x3[:steps-1])[:10])

    print(diffed[:,1])
    print(np.diff(x2[:-11]))
    print(diffed_2[:,1])
    print(np.diff(x3[:steps-1]))


    # ax5 = plt.subplot(324)
    # plt.title("calculated error using /joint_traj")
    # plt.plot(x3[:steps],joint_traj-arm_traj_desired, linewidth = 2)
    # ax5.legend(names, loc = "best")
    # ax5.grid(True, linestyle='-.')

    
    plt.show()