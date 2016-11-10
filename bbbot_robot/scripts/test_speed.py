#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
from sensor_msgs.msg import JointState
import time

start_time = 0
start_value = 0
prev_value = 0


def joint_state_cb(data):
    global start_time, start_value, prev_value
    curr_value = data.position[data.name.index('leftarm_wrist_3_joint')]
    # curr_value = data.position[data.name.index('leftarm_shoulder_pan_joint')]
    if start_value == 0:
        start_value = curr_value

    if (start_time == 0) and np.abs(start_value - curr_value) > 0.01:
        start_time = time.time()
        print("start value", curr_value)
    elif start_time and np.abs(prev_value - curr_value) < 0.001:
        # print "Inside", np.abs(curr_value)
        # if (np.abs(curr_value) < 0.1 or np.abs(curr_value) > 1.5):
        if np.abs(curr_value) > 3.05:
            t = (time.time() - start_time)
            # print("time", (time.time() - start_time) * 4, start_value, curr_value)
            print "Time: ", t, "RPM: ", 60.0 / t
            print "\t", start_value, curr_value, prev_value, np.abs(prev_value - curr_value)
            start_time = 0
            start_value = curr_value

    prev_value = curr_value


def main():
    if len(sys.argv) < 2:
        return

    rospy.init_node("basketball_robot")
    rospy.Subscriber("/leftarm/joint_states", JointState, joint_state_cb)
    robot = Robot(use_prefix=True, single_arm=True, pos_controller=True)

    delay = 0.01
    inc = 3
    # robot.control_torque()

    if sys.argv[1] == '1':
        # robot.create_trajectory('left', JN.SHOULDER_PAN, 0, inc, delay)
        robot.create_trajectory('left', JN.WRIST_3, -np.pi, inc, delay)
    elif sys.argv[1] == '2':
        # robot.create_trajectory('left', JN.SHOULDER_PAN, -np.pi / 2, inc, delay)
        robot.create_trajectory('left', JN.WRIST_3, np.pi, inc, delay)
    else:
        return

    robot.start_trajectory(2)

    while robot.left.trajectory_active:
        pass
    print("time", time.time() - start_time)


def main_2():
    if len(sys.argv) < 2:
        return

    rospy.init_node("basketball_robot")
    robot = Robot(use_prefix=True, pos_controller=True)
    robot.control_torque()

    delay = 0.01
    inc = 0.06

    if sys.argv[1] == '1':
        robot.create_trajectory('left', JN.SHOULDER_PAN, -0.2, inc, delay)
        robot.create_trajectory('right', JN.SHOULDER_PAN, 0.2, inc, delay)
        # robot.create_trajectory('left', JN.WRIST_3, -np.pi, inc, delay)
        # robot.create_trajectory('right', JN.WRIST_3, -np.pi, inc, delay)
    elif sys.argv[1] == '2':
        robot.create_trajectory('left', JN.SHOULDER_PAN, -np.pi / 2, inc, delay)
        robot.create_trajectory('right', JN.SHOULDER_PAN, np.pi / 2, inc, delay)
        # robot.create_trajectory('left', JN.SHOULDER_PAN, -np.pi / 2, inc, delay)
        # robot.create_trajectory('left', JN.WRIST_3, np.pi, inc, delay)
        # robot.create_trajectory('right', JN.WRIST_3, np.pi, inc, delay)
    else:
        return

    robot.start_trajectory(2)
    robot.wait_completion()


if __name__ == '__main__':
    # main()
    main_2()
