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

old_time = None
old_position = None
vel_position = []
vel = []


def joint_state_cb(data):
    global old_time, old_position, vel_position, vel

    idx = data.name.index('leftarm_wrist_3_joint')
    curr_position = data.position[idx]

    if old_time is not None:
        pos_diff = curr_position - old_position
        t_diff = data.header.stamp - old_time
        vel_position.append(float(pos_diff) / t_diff.to_sec())
        vel.append(data.velocity[idx])

    old_time = data.header.stamp
    old_position = curr_position


def save_data():
    print ("shutting down")
    data = np.array([vel_position, vel])
    np.save('velocity.data', data)


def main():
    rospy.init_node("basketball_robot")
    rospy.Subscriber("/leftarm/joint_states", JointState, joint_state_cb)
    robot = Robot(use_prefix=True, single_arm=True)
    robot.control_torque()

    rospy.on_shutdown(save_data)
    delay = 0.1

    # robot.create_trajectory('left', JN.WRIST_3, -np.pi, 0.06, delay)
    robot.create_trajectory('left', JN.SHOULDER_PAN, np.pi, 0.06, delay)
    # robot.create_trajectory('left', JN.SHOULDER_PAN, np.pi / 2, 0.06, delay)

    robot.start_trajectory(2)

    while robot.left.trajectory_active:
        pass

    time.sleep(2)


if __name__ == '__main__':
    main()
