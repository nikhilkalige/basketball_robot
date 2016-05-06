#! /usr/bin/env python
import rospy
import sys
import numpy as np
from bbbot_robot.robot import Robot
from bbbot_robot.robot import JointNames as JN


p1 = [-0.22, 0.82, -0.341, 0, 1.34, -1.5,
      0.22, 0.82, -0.341, 0, 1.34, -1.5]

p2 = [-0.22, 1.256, -0.341, 0, 1.34, -1.5,
      0.22, 1.256, -0.341, 0, 1.34, -1.5]


def move_to_point(robot, pt):
    delay = 0.2

    for pos in xrange(6):
        robot.create_trajectory('left', pos, pt[pos], 0.05, delay)
        robot.create_trajectory('right', pos, pt[pos + 6], 0.05, delay)
        robot.start_trajectory(2)
        robot.wait_completion()


def move_to_pos(robot, joint, leftpos, rightpos, delay):
    robot.create_trajectory('left', joint, leftpos, 0.05, delay)
    robot.create_trajectory('right', joint, rightpos, 0.05, delay)
    robot.start_trajectory(2)
    robot.wait_completion()


def main():
    rospy.init_node("basketball_robot")
    robot = Robot(use_prefix=True)

    delay = 0.2

    robot.control_torque()

    # Move to initial position
    # move_to_point(robot, p1)

    # Close hands
    # move_to_pos(robot, JN.SHOULDER_LIFT, 1.256, 1.256, delay)

    # Throw ball
    nextpt = 1.8
    move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, delay)
    # Move Faster now
    nextpt = 3.2
    move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, .04)

    # Back to start
    # nextpt = 0.22
    nextpt = 1.8
    move_to_pos(robot, JN.SHOULDER_PAN, -nextpt, nextpt, delay)

    # Open hands
    # move_to_pos(robot, JN.SHOULDER_LIFT, 0.82, 0.82, delay)


if __name__ == '__main__':
    main()
