#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN
from bbbot_robot.tracking import Tracker

def main():
    robot = Robot(use_prefix=True)

    delay = 0.2

    robot.control_torque(False)
    # for i in xrange(2):
    #     robot.create_trajectory('left', JN.SHOULDER_PAN, .05, 0.05, delay)
    #     robot.create_trajectory('right', JN.SHOULDER_PAN, -.45, -0.05, delay)
    #     robot.start_trajectory(2)
    #     robot.wait_completion()

    #     robot.create_trajectory('left', JN.SHOULDER_PAN, -.45, -0.05, delay)
    #     robot.create_trajectory('right', JN.SHOULDER_PAN, .05, 0.05, delay)
    #     robot.start_trajectory(2)
    #     robot.wait_completion()

    rospy.spin()


def tracker():
    t = Tracker()
    print("Starting thread")
    while True:
        t.start()
        raw_input('Enter to stop: ')
        t.stop()
        print 'Reward', t.get_reward()
        raw_input('Enter to start: ')


if __name__ == '__main__':
    rospy.init_node("basketball_robot")
    # main()
    tracker()
