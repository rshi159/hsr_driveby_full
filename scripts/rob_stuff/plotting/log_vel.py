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
arm_traj_actual = np.array([[0,0,0,0,0]])
arm_traj_desired = np.array([[0,0,0,0,0]])
arm_traj_error = np.array([[0,0,0,0,0]])
joint_traj = np.array([[0,0,0,0,0]])
cb2 = False
name = np.array([])

base_traj_actual = np.array([[0,0,0]])
base_traj_desired = np.array([[0,0,0]])
base_traj_error = np.array([[0,0,0]])
base_traj = np.array([[0,0,0]])

# ==========USER INPUTS===========
# number of datapoints to read in. 100 * dt * 10
steps = 100
# plot position or velocity
plot_positions = False
# number of histogram bins
n_bins = 25
# plot base
plot_base = False
# ================================

def callback_2(data):
    global x2
    global cb2
    global arm_traj_actual
    global arm_traj_desired
    global arm_traj_error
    global base_traj_actual
    global base_traj_desired
    global base_traj_error
    global steps
    global plot_base
    if len(arm_traj_desired) > steps:
        rospy.signal_shutdown("Finished")
    if not cb2:
        cb2 = True
        print("STARTING")
    if plot_base:
        if plot_positions:
            base_traj_actual = np.append(base_traj_actual, [data.feedback.actual.positions], axis = 0)
            base_traj_desired = np.append(base_traj_desired, [data.feedback.desired.positions], axis = 0)
            base_traj_error = np.append(base_traj_error, [data.feedback.error.positions], axis = 0)
        else:
            arm_traj_actual = np.append(base_traj_actual, [data.feedback.actual.velocities], axis = 0)
            arm_traj_desired = np.append(base_traj_desired, [data.feedback.desired.velocities], axis = 0)
            arm_traj_error = np.append(base_traj_error, [data.feedback.error.velocities], axis = 0)
    else:
        if plot_positions:
            arm_traj_actual = np.append(arm_traj_actual, [data.feedback.actual.positions], axis = 0)
            arm_traj_desired = np.append(arm_traj_desired, [data.feedback.desired.positions], axis = 0)
            arm_traj_error = np.append(arm_traj_error, [data.feedback.error.positions], axis = 0)
        else:
            arm_traj_actual = np.append(arm_traj_actual, [data.feedback.actual.velocities], axis = 0)
            arm_traj_desired = np.append(arm_traj_desired, [data.feedback.desired.velocities], axis = 0)
            arm_traj_error = np.append(arm_traj_error, [data.feedback.error.velocities], axis = 0)
    # print(np.shape(arm_traj_desired)[0])
    time = data.feedback.header.stamp.secs + data.feedback.header.stamp.nsecs * 1e-9
    # print(time)
    x2 = np.append(x2, time)

def callback_3(data):
    global x3
    global joint_traj
    global cb2
    global steps
    global names
    if cb2:
        if len(joint_traj) > steps+5:
            rospy.signal_shutdown("Finished")
        # print("=====cb 3=====")
        # print(data.position[14:19])
        time = data.header.stamp.secs + data.header.stamp.nsecs * 1e-9
        # print("===="+str(time))
        x3 = np.append(x3, time)
        if plot_base:
            if plot_positions:
                joint_traj = np.append(joint_traj, [data.position[0:3]], axis = 0)
            else:
                joint_traj = np.append(joint_traj, [data.velocity[0:3]], axis = 0)
            names = data.name[0:3]
        else:
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
    if plot_base:
        # rospy.Subscriber('/hsrb/omni_base_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, callback_1)
        rospy.Subscriber('/hsrb/omni_base_trajectory_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback,callback_2)
    else:
        # rospy.Subscriber('/hsrb/omni_base_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback, callback_1)
        rospy.Subscriber('/hsrb/arm_trajectory_controller/follow_joint_trajectory/feedback', FollowJointTrajectoryActionFeedback,callback_2)
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
    if cb2_len < steps-1 or cb3_len < steps-1:
        steps = np.minimum(cb2_len, cb3_len)
        print("changed number of steps due to lost data: "+ str(steps))
    # print(arm_traj_actual)
    # print(np.delete(arm_traj_actual,0,0))
    x2 = x2[:steps-1]
    x3 = x3[:steps-1]
    if plot_base:
        base_traj_actual = np.delete(base_traj_actual,0,0)[:steps-1]
        base_traj_desired = np.delete(base_traj_desired,0,0)[:steps-1]
        base_traj_error = np.delete(base_traj_error,0,0)[:steps-1]
        base_traj = np.delete(joint_traj,0,0)[:steps-1]

        ax1 = plt.subplot(321)#,sharex=ax2, sharey=ax2)
        plt.title("/base_feedback actual")
        plt.plot(x2[:steps-1], base_traj_actual[:steps-1], linewidth = 2)
        ax1.grid(True, linestyle='-.')
        # ax1.set_ylim((-1, 1))

        ax2 = plt.subplot(322)#,sharex=ax1, sharey=ax1)
        plt.title("/base_feedback desired")
        plt.plot(x2[:steps-1], base_traj_desired[:steps-1], linewidth = 2)
        ax2.grid(True, linestyle='-.')
        ax2.legend(names, loc = 'best')

        ax3 = plt.subplot(323,sharex=ax2, sharey=ax2)
        plt.title("/joint_traj")
        plt.plot(x3[:steps-1], joint_traj[:steps-1], linewidth = 2)
        ax3.grid(True, linestyle='-.')

        ax4 = plt.subplot(324)
        plt.title("/feedback error")
        plt.plot(x2[:steps-1],base_traj_error[:steps-1], linewidth = 2)
        ax4.grid(True, linestyle='-.')
        
        print("=====")
        print(np.shape(x2[:steps-1]))
        print(np.shape(joint_traj[:steps-1]))
        print(np.shape(base_traj_desired[:steps-1]))
        print(np.shape(joint_traj[:steps-1]-base_traj_desired[:steps-1]))

        ax5 = plt.subplot(325)
        plt.title("/joint_traj error")
        plt.plot(x2[:steps-1], joint_traj[:steps-1]-base_traj_desired[:steps-1], linewidth = 2)
        ax5.grid(True, linestyle='-.')


        # ax5 = plt.subplot(324)
        # plt.title("calculated error using /joint_traj")
        # plt.plot(x3[:steps],joint_traj-arm_traj_desired, linewidth = 2)
        # ax5.legend(names, loc = "best")
        # ax5.grid(True, linestyle='-.')

        # ax6 = plt.subplot(326)
        # plt.title("histograms for arm_roll_joint")
        # ax6.hist(joint_traj[:steps-1,2]-base_traj_desired[:steps-1,2], bins=n_bins, color = 'b', alpha=0.5, label='joint - desired')
        # ax6.hist(arm_traj_error[:steps-1,2], bins=n_bins,color = 'r', alpha=0.5, label='arm_error')
        
    else:
        arm_traj_actual = np.delete(arm_traj_actual,0,0)[:steps-1]
        arm_traj_desired = np.delete(arm_traj_desired,0,0)[:steps-1]
        arm_traj_error = np.delete(arm_traj_error,0,0)[:steps-1]
        joint_traj = np.delete(joint_traj,0,0)[:steps-1]
        # print(x2[:steps])
        # print(arm_traj_actual)
        print(len(x2))
        print(len(x3))
        print(len(arm_traj_actual))
        print(len(arm_traj_desired))
        print(len(arm_traj_error))
        print(len(joint_traj))
        print(len(joint_traj - arm_traj_desired))

        # save the expected velocity trajectory for plotting with my_acceleration.py
        # np.save('expected_velocity',arm_traj_desired[:steps-1])
        # exit()


        ax1 = plt.subplot(321)#,sharex=ax2, sharey=ax2)
        plt.title("/arm_feedback actual")
        plt.plot(x2[:steps-1], arm_traj_actual[:steps-1], linewidth = 2)
        ax1.grid(True, linestyle='-.')
        # ax1.set_ylim((-1, 1))

        ax2 = plt.subplot(322)#,sharex=ax1, sharey=ax1)
        plt.title("/arm_feedback desired")
        plt.plot(x2[:steps-1], arm_traj_desired[:steps-1], linewidth = 2)
        ax2.grid(True, linestyle='-.')
        ax2.legend(names, loc = 'best')

        ax3 = plt.subplot(323,sharex=ax2, sharey=ax2)
        plt.title("/joint_traj")
        plt.plot(x3[:steps-1], joint_traj[:steps-1], linewidth = 2)
        ax3.grid(True, linestyle='-.')

        ax4 = plt.subplot(324)
        plt.title("/feedback error")
        plt.plot(x2[:steps-1],arm_traj_error[:steps-1], linewidth = 2)
        ax4.grid(True, linestyle='-.')
        
        print("=====")
        print(np.shape(x2[:steps-1]))
        print(np.shape(joint_traj[:steps-1]))
        print(np.shape(arm_traj_desired[:steps-1]))
        print(np.shape(joint_traj[:steps-1]-arm_traj_desired[:steps-1]))

        ax5 = plt.subplot(325)
        plt.title("/joint_traj error")
        plt.plot(x2[:steps-1], joint_traj[:steps-1]-arm_traj_desired[:steps-1], linewidth = 2)
        ax5.grid(True, linestyle='-.')


        # ax5 = plt.subplot(324)
        # plt.title("calculated error using /joint_traj")
        # plt.plot(x3[:steps],joint_traj-arm_traj_desired, linewidth = 2)
        # ax5.legend(names, loc = "best")
        # ax5.grid(True, linestyle='-.')

        ax6 = plt.subplot(326)
        plt.title("histograms for arm_roll_joint")
        ax6.hist(joint_traj[:steps-1,2]-arm_traj_desired[:steps-1,2], bins=n_bins, color = 'b', alpha=0.5, label='joint - desired')
        ax6.hist(arm_traj_error[:steps-1,2], bins=n_bins,color = 'r', alpha=0.5, label='arm_error')
        #first 100 points
        # ax6.hist(joint_traj[:100,2]-arm_traj_desired[:100,2], bins=n_bins, color = 'b', alpha=0.5, label='joint - desired')
        # ax6.hist(arm_traj_error[:100,2], bins=n_bins,color = 'r', alpha=0.5, label='arm_error')
        
        # time differences
        # plt.title("time difference /feedback - /joint_traj")
        # ax6.hist(x2[:steps-1]-x3[:steps-1], bins=n_bins,color = 'r', alpha=0.5, label='time diff')
        # ax6.grid(True, linestyle='-.')
        # ax6.legend(loc = 'best')
        # ax6.set(xlabel = "seconds, bins: " + str(n_bins), ylabel="counts")
        # print("saving trajectory")
        # np.save('times',[x2,x3,x2[:steps-1]-x3[:steps-1]])

    
    plt.show()